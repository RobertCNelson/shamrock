/******************************************************************************
 * Copyright (c) 2013-2014, Texas Instruments Incorporated - http://www.ti.com/
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *       * Redistributions of source code must retain the above copyright
 *         notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *         notice, this list of conditions and the following disclaimer in the
 *         documentation and/or other materials provided with the distribution.
 *       * Neither the name of Texas Instruments Incorporated nor the
 *         names of its contributors may be used to endorse or promote products
 *         derived from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 *   THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include "wga.h"
#include <iostream>
#include <llvm/Pass.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/InstIterator.h>
#include <llvm/IR/IntrinsicInst.h>
#include "llvm/Support/CFG.h"
#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/UnifyFunctionExitNodes.h"
#include "boost/assign/std/set.hpp"
#include <stdio.h>

using namespace std;
using namespace boost::assign;

namespace llvm
{

/******************************************************************************
* createTIOpenclWorkGroupAggregation
******************************************************************************/
Pass *createTIOpenclWorkGroupAggregationPass(bool is_pocl_mode) 
{
    TIOpenclWorkGroupAggregation *fp = new TIOpenclWorkGroupAggregation(
                                                                 is_pocl_mode);
    return fp;
}

/**************************************************************************
* Constructor
**************************************************************************/
TIOpenclWorkGroupAggregation::TIOpenclWorkGroupAggregation(bool pocl_mode) : 
    FunctionPass(ID), is_pocl_mode(pocl_mode)
{
    for (int i = 0; i < MAX_DIMENSIONS; ++i) IVPhi[i] = 0;
}

/**************************************************************************
* Get index variable
* 1. Original mode, only one loop inserted: return IVPhi[]
* 2. pocl mode, multiple loops inserted: return a new LoadInst
**************************************************************************/
llvm::Instruction* TIOpenclWorkGroupAggregation::get_IV(Function &F, 
                                                        CallInst *call)
{
    llvm::Value *ivx, *ivy, *ivz;
    Value *arg = call->getArgOperand(0);
    uint32_t dim = 9999;

    if (ConstantInt * constInt = dyn_cast<ConstantInt>(arg))
        dim = constInt->getSExtValue();

    if (is_pocl_mode)
    {
        llvm::GlobalValue *iv;
        if (dim == 2)
            iv = F.getParent()->getNamedGlobal("_local_id_z");
        else if (dim == 1)
            iv = F.getParent()->getNamedGlobal("_local_id_y");
        else if (dim == 0)
            iv = F.getParent()->getNamedGlobal("_local_id_x");
        if (dim != 9999) return new LoadInst(iv);

        ivx = F.getParent()->getNamedGlobal("_local_id_x");
        ivy = F.getParent()->getNamedGlobal("_local_id_y");
        ivz = F.getParent()->getNamedGlobal("_local_id_z");
    }
    else
    {
        if (dim != 9999) return IVPhi[dim];

        ivx = IVPhi[0];
        ivy = IVPhi[1];
        ivz = IVPhi[2];
    }

    // not constant arg: return (arg == 2) ? ivz : (arg == 1 ? ivy : ivx)
    Type        *Int32 = Type::getInt32Ty(F.getContext());
    Value       *one   = ConstantInt::get(Int32, 1); 
    Value       *two   = ConstantInt::get(Int32, 2); 
    llvm::Value *cyx = new ICmpInst(call, ICmpInst::ICMP_EQ, arg, two);
    llvm::Value *syx = SelectInst::Create(cyx, ivy, ivx, "", call);
    llvm::Value *czyx = new ICmpInst(call, ICmpInst::ICMP_EQ, arg, one);
    return SelectInst::Create(czyx, ivz, syx, "", is_pocl_mode ? NULL : call);
}

/**************************************************************************
* runOnFunction(Function &F) 
**************************************************************************/
bool TIOpenclWorkGroupAggregation::runOnFunction(Function &F) 
{
    /*-------------------------------------------------------------------------
    * Determine how many dimensions are referenced using OpenCL getXXX 
    * functions, and record them all for later rewrite.
    *------------------------------------------------------------------------*/
    int dims;
    if (!is_pocl_mode)  dims = findNeededLoopNest(F);

    /*-------------------------------------------------------------------------
    * Add a loop nest for each dimension referenced that requires a workitem
    * id.
    *------------------------------------------------------------------------*/
    if (!is_pocl_mode)  for (int i = 0; i < dims; ++i) add_loop(F, i); 

    /*-------------------------------------------------------------------------
    * rewrite the alloca() generated during pocl llvm work-group aggregation
    *------------------------------------------------------------------------*/
    if (is_pocl_mode)  rewrite_allocas(F);

    /*-------------------------------------------------------------------------
    * rewrite the OpenCL getXXX dimension query functions to reference the info
    * packet for the workgroup.  Return true if we modified the function.
    *------------------------------------------------------------------------*/
    return rewrite_ocl_funcs(F);
}

/******************************************************************************
* getAnalysisUsage(AnalysisUsage &Info) const
******************************************************************************/
void TIOpenclWorkGroupAggregation::getAnalysisUsage(AnalysisUsage &Info) const
{
    /*-------------------------------------------------------------------------
    * This will ensure that all returns go through a single exit node, which 
    * our WGA loop generation algorithm depends on.
    *------------------------------------------------------------------------*/
    Info.addRequired<UnifyFunctionExitNodes>();
}

/**************************************************************************
* findNeededLoopNest(Function &F) 
**************************************************************************/
unsigned int TIOpenclWorkGroupAggregation::findNeededLoopNest(Function &F) 
{
    unsigned int maxDim = 0;

    for (inst_iterator I = inst_begin(&F), E = inst_end(&F); I != E; ++I)
        if (CallInst * callInst = dyn_cast<CallInst>(&*I))
        {
            if (!callInst->getCalledFunction()) continue;
            string functionName(callInst->getCalledFunction()->getName());

            if (functionName == "get_local_id" || 
                functionName == "get_global_id")
            {
                Value *arg = callInst->getArgOperand(0);
                if (ConstantInt * constInt = dyn_cast<ConstantInt>(arg))
                {
                    unsigned int dimIdx = constInt->getSExtValue();
                    dimIdx = min(MAX_DIMENSIONS-1, dimIdx); 
                    maxDim = max(maxDim, dimIdx + 1);
                }

                /*-------------------------------------------------------------
                * if the work group function has a variable argument, then 
                * assume worst case and return 3 loop levels are needed.
                *------------------------------------------------------------*/
                else return 3;
            }
        }

    return maxDim;
}

/**************************************************************************
* createLoadGlobal
*     Create an aligned 32 bit load from a global address.
**************************************************************************/
Instruction* TIOpenclWorkGroupAggregation::createLoadGlobal
    (int32_t idx, Module* M, Instruction *before, const char *name)
{
    llvm::ArrayType *type = ArrayType::get(
    IntegerType::getInt32Ty(getGlobalContext()), 64);
    llvm::Value* dummy     = M->getOrInsertGlobal("kernel_config_l2", type);

    GlobalVariable* global = M->getNamedGlobal("kernel_config_l2"); 

    std::vector<Value*> indices;
    indices.push_back(ConstantInt::get(IntegerType::getInt32Ty(getGlobalContext()), 0));
    indices.push_back(ConstantInt::get(IntegerType::getInt32Ty(getGlobalContext()), idx));

    Constant* gep = ConstantExpr::getInBoundsGetElementPtr (global, indices);
    LoadInst* ld  = new LoadInst(gep, name, before);

    ld->setAlignment(4);
    return ld;
}

/******************************************************************************
* findDim
******************************************************************************/
unsigned int TIOpenclWorkGroupAggregation::findDim(class CallInst* call)
{
    Value *arg = call->getArgOperand(0);

    if (ConstantInt * constInt = dyn_cast<ConstantInt>(arg))
        return constInt->getSExtValue();
    return 100;  // who knows
}

/**************************************************************************
* rewrite allocas to _wg_alloca(sizeinbytes)
**************************************************************************/
bool TIOpenclWorkGroupAggregation::rewrite_allocas(Function &F) 
{
    int wi_alloca_size = 0;
    Module *M = F.getParent();
    AllocaInst *alloca;

    std::vector<AllocaInst *> allocas;
    for (inst_iterator I = inst_begin(&F), E = inst_end(&F); I != E; ++I)
        if ((alloca = dyn_cast<AllocaInst>(&*I)) != NULL)
            allocas.push_back(alloca);
    if (allocas.empty()) return false;

    DataLayout dataLayout(M);
    FunctionType *ft = FunctionType::get
            (/*Result=*/   IntegerType::get(M->getContext(), 32),
             /*Params=*/   IntegerType::get(M->getContext(), 32),
             /*isVarArg=*/ false);
    Function *wg_alloca = dyn_cast<Function>(
                                    M->getOrInsertFunction("_wg_alloca", ft));
    Type *Int32 = Type::getInt32Ty(M->getContext());

    for (std::vector<AllocaInst *>::iterator I = allocas.begin();
         I != allocas.end(); ++I)
    {
        alloca = *I;

        // get number of elements, element type size, compute total size
        Value *numElems = alloca->getArraySize();
        // YUAN TODO: skip regular constant numElems?

        Type *elementType = alloca->getAllocatedType();
        // getTypeSizeInBits(), what about uchar3 type?
        uint64_t esBytes = dataLayout.getTypeStoreSize(elementType);
        Value *esize = ConstantInt::get(Int32, (uint32_t) esBytes); 
        Instruction *alloca_size = BinaryOperator::Create(
                               Instruction::Mul, esize, numElems, "", alloca);
        SmallVector<Value *, 4> args;
        args.push_back(alloca_size);
        
        // create function call: _wg_alloca(alloca_size)
        CallInst *f_alloca = CallInst::Create(
                              wg_alloca, ArrayRef<Value *>(args), "", alloca);

        // cast to alloca type
        Instruction * new_alloca = new IntToPtrInst(
                                                 f_alloca, alloca->getType());

        // replace AllocaInst with new _wg_alloca()
        ReplaceInstWithInst(alloca, new_alloca);

        // accumulate element type size
        unsigned align = dataLayout.getPrefTypeAlignment(elementType);
        wi_alloca_size = (wi_alloca_size + align - 1) & (~(align-1));
        wi_alloca_size += esBytes;
    }

    // initialize _wg_alloca_start and _wg_alloca_size
    // _wg_alloca_size  = load(packetaddr+offset);
    // _wg_alloca_start = load(packetaddr+offset) + __core_num() * _wg_alloca_size;
    Instruction *inspt = F.getEntryBlock().getFirstNonPHI();
    FunctionType *core_num_ft = FunctionType::get
            (/*Result=*/   IntegerType::get(M->getContext(), 32),
             /*isVarArg=*/ false);
    Function *core_num = dyn_cast<Function>(
                         M->getOrInsertFunction("__core_num", core_num_ft));
    Instruction *f_core_num = CallInst::Create(core_num, "", inspt);

    Instruction *wg_alloca_size = createLoadGlobal(17, M, inspt);

    Instruction *shift = BinaryOperator::Create(Instruction::Mul, f_core_num,
                                                wg_alloca_size, "", inspt);

    Instruction *start = createLoadGlobal(16, M, inspt);

    Instruction *core_start = BinaryOperator::Create(
                               Instruction::Add, start, shift, "", inspt);
    Value *gv = M->getOrInsertGlobal("_wg_alloca_start", Int32);
    GlobalVariable *wg_gv = M->getNamedGlobal("_wg_alloca_start");
    wg_gv->setSection(StringRef("far"));
    Instruction *store = new StoreInst(core_start, gv, inspt);

    // put total orig_wi_size into attributes data in the function
    char *s_wi_alloca_size = new char[32];  // we have to leak this
    snprintf(s_wi_alloca_size, 32, "_wi_alloca_size=%d", wi_alloca_size);
    F.addFnAttr(StringRef(s_wi_alloca_size));

    return true;
}

/**************************************************************************
* rewrite_ocl_funcs
**************************************************************************/
bool TIOpenclWorkGroupAggregation::rewrite_ocl_funcs(Function &F) 
{
    CallInst *call;
    Module *M = F.getParent();
    std::vector<CallInst *> wi_calls;
    for (inst_iterator I = inst_begin(&F), E = inst_end(&F); I != E; ++I)
    {
        if ((call = dyn_cast<CallInst>(&*I)) == NULL)           continue;
        if (call->getCalledFunction() == NULL)                  continue;
        string name(call->getCalledFunction()->getName());
        if (name != "get_local_id" && name != "get_local_size") continue;
        wi_calls.push_back(call);
    }
    if (wi_calls.empty()) return false;
    
    LLVMContext &ctx = F.getContext();
    std::vector<CallInst *>::iterator I, E;
    for (I = wi_calls.begin(), E = wi_calls.end(); I != E; ++I)
    {
        call = *I;
        string name(call->getCalledFunction()->getName());

        if (name == "get_local_id") 
        {
            if (is_pocl_mode)
            {
                ReplaceInstWithInst(call, get_IV(F, call));
            }
            else
            {
                BasicBlock::iterator BI(call);
                ReplaceInstWithValue(call->getParent()->getInstList(), BI,
                                     get_IV(F, call));
            }
        }
        else if (name == "get_local_size")
        {
            // remaining get_local_size() are generated by pocl,
            // arguments guaranteed to be constants: 0, 1, or 2
            ReplaceInstWithInst(call,
                                createLoadGlobal(4+findDim(call), M));
        }
    }
    return true;
}

BasicBlock* TIOpenclWorkGroupAggregation::findExitBlock(Function &F)
{
    BasicBlock *exit = 0;

    /*-------------------------------------------------------------------------
    * Find the one block with no successors
    *------------------------------------------------------------------------*/
    for (Function::iterator B = F.begin(), E = F.end(); B != E; ++B)
        if ((*B).getTerminator()->getNumSuccessors() == 0) 
            if (!exit) exit = &(*B);
            else assert(false);

    /*-------------------------------------------------------------------------
    * Split the return off into it's own block
    *------------------------------------------------------------------------*/
    Instruction *ret = exit->getTerminator();

    if (ret != &exit->front())
        exit = SplitBlock(exit, ret, this);

    return exit;
}

/**************************************************************************
* add_loop(Function &F)
**************************************************************************/
void TIOpenclWorkGroupAggregation::add_loop(Function &F, int dimIdx)
{
   LLVMContext &ctx   = F.getContext();
   Type        *Int32 = Type::getInt32Ty(ctx);
   Value       *zero  = ConstantInt::get(Int32, 0); 
   Value       *one   = ConstantInt::get(Int32, 1); 
   Module      *M     = F.getParent();

   BasicBlock*  exit     = findExitBlock(F);
   BasicBlock*  entry    = &(F.getEntryBlock());

   BasicBlock*  bodytop  = SplitBlock(entry, &entry->front(), this);
   BasicBlock*  bodyend  = exit;
                exit     = SplitBlock(bodyend, &exit->front(), this);

   exit->setName(".exit");
   entry->setName(".entry");
   bodytop->setName(".bodyTop");
   bodyend->setName(".bodyEnd");

   /*----------------------------------------------------------------------
   * Populate the branch around
   *---------------------------------------------------------------------*/
   Instruction *branch = entry->getTerminator();
   Instruction *ld_upper_bnd = createLoadGlobal(4+dimIdx, M, branch);

   Instruction *cmp = CmpInst::Create (Instruction::ICmp, CmpInst::ICMP_SGT, 
                           ld_upper_bnd, zero, "", branch);

   Instruction *cbr = BranchInst::Create(bodytop, exit, cmp);
   ReplaceInstWithInst(branch, cbr);

   /*----------------------------------------------------------------------
   * Add the phi node to the top of the body
   *---------------------------------------------------------------------*/
   PHINode *phi = PHINode::Create(Int32, 0, "", &bodytop->front());
   phi->addIncoming(zero, entry);

   /*----------------------------------------------------------------------
   * Add the loop control to the bottom of the bodyend
   *---------------------------------------------------------------------*/
   branch = bodyend->getTerminator();
   Instruction *inc = BinaryOperator::Create(Instruction::Add, 
                      phi, one, Twine(), branch);

   Instruction *ld_upper_bnd2 = createLoadGlobal(4+dimIdx, M, branch);
   Instruction *cmp2 = CmpInst::Create (Instruction::ICmp, CmpInst::ICMP_SLT, 
                       inc, ld_upper_bnd2, "", branch);

   Instruction *cbr2 = BranchInst::Create(bodytop, exit, cmp2);
   ReplaceInstWithInst(branch, cbr2);

   phi->addIncoming(inc, bodyend);
   IVPhi[dimIdx] = phi;

   // YUAN TODO: maybe handled better later
   if (dimIdx < 1) IVPhi[1] = phi;
   if (dimIdx < 2) IVPhi[2] = phi;
}

char TIOpenclWorkGroupAggregation::ID = 0;
static RegisterPass<TIOpenclWorkGroupAggregation> 
                   X("wga", "Work Group Aggregation", false, false);

}

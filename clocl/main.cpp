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
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <sys/stat.h>

#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/PassManager.h>
#include <llvm/Analysis/Passes.h>
#include <llvm/Analysis/Verifier.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/IPO.h>
#include <llvm/Transforms/Utils/UnifyFunctionExitNodes.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/Support/MemoryBuffer.h>
#include <llvm/Support/Casting.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Linker.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/Support/InstIterator.h>

#include "compiler.h"
#include "wga.h"
#include "file_manip.h"
#include "options.h"

#include <WorkitemHandlerChooser.h>
#include <BreakConstantGEPs.h>
#include <Flatten.h>
#include <PHIsToAllocas.h>
#include <IsolateRegions.h>
#include <VariableUniformityAnalysis.h>
#include <ImplicitLoopBarriers.h>
#include <LoopBarriers.h>
#include <BarrierTailReplication.h>
#include <CanonicalizeBarriers.h>
#include <WorkItemAliasAnalysis.h>
#include <WorkitemReplication.h>
#include <WorkitemLoops.h>
#include <AllocasToEntry.h>
#include <Workgroup.h>
#include <TargetAddressSpaces.h>

using namespace std;
using llvm::Module;

bool    prepend_headers(string filename, string& source);
bool    run_clang      (string filename, string source, Compiler &compiler, 
                        Module **module, string& binary);
bool    llvm_xforms    (Module *module, bool optimize);
bool    cl6x           (string& filename, string &binary_str);

int     run_cl6x       (string filename, string *llvm_bitcode, string options);
void    write_binary   (string filename, const char *buf, int size);
void    write_text     (string filename);


void write_bitcode(string filename, Module* module)
{
    string bc_file(fs_filename(fs_replace_extension(filename, ".bc")));
    string err_info;

    llvm::raw_fd_ostream file_ostream(bc_file.c_str(), err_info);
    llvm::WriteBitcodeToFile(module, file_ostream);
    file_ostream.flush();
}

/******************************************************************************
* main
******************************************************************************/
int main(int argc, char *argv[])
{
    process_options(argc, argv);

    if (files_clc.empty()) return 0;

    string    filename = files_clc[0];
    string    source;  // OpenCL C program source
    string    binary;  // Untransformed LLVM bitcode (still per workitem)
    Module   *module;  // module, evolves during transformation
    Compiler  compiler;

    if (!prepend_headers(filename, source))                            exit(-1);
    if (!run_clang      (filename, source, compiler, &module, binary)) exit(-1);
    if (!llvm_xforms    (module, compiler.optimize()))                 exit(-1);

    write_bitcode(filename, module);

    if (!cl6x (filename, binary))                                      exit(-1);

    if (opt_txt) write_text(filename);
}



/******************************************************************************
* prepend_headers
******************************************************************************/
bool prepend_headers(string filename, string& source)
{
    /*---------------------------------------------------------------------
    * Compile the Kernel Source for the device
    *--------------------------------------------------------------------*/
    if (!fs_exists(filename))
    { cout << "File " << filename << " doesn't exist" << endl; return false; }

    stringstream userSrc;
    userSrc << ifstream(filename.c_str()).rdbuf();

    /*-------------------------------------------------------------------------
    * Prepend OpenCL header info into the source
    *------------------------------------------------------------------------*/
    source = userSrc.str();
    return true;
}

/******************************************************************************
* run_clang
******************************************************************************/
bool run_clang(string filename, string source, Compiler &compiler, 
           Module **module, string& binary)
{
    using llvm::MemoryBuffer;
    using llvm::StringRef;

    const StringRef s_data(source);
    const StringRef s_name("<source>");

    MemoryBuffer *buffer = MemoryBuffer::getMemBuffer(s_data, s_name);

    if (opt_verbose) cout << "clang options: " << cl_options << endl;
    if (!compiler.compile(cl_options, buffer, filename)) 
        return false;

    *module = compiler.module();

    llvm::raw_string_ostream str_ostream(binary);
    llvm::WriteBitcodeToFile(*module, str_ostream);
    str_ostream.flush();

    return true;
}

/******************************************************************************
* llvm_xforms
******************************************************************************/
bool llvm_xforms(Module *module, bool optimize)
{
    // Get list of kernels to strip other unused functions
    vector<const char *> api;
    vector<string> api_s;     // Needed to keep valid data in api

    llvm::NamedMDNode *kern_meta = module->getNamedMetadata("opencl.kernels");

    for (unsigned int i=0; kern_meta && i < kern_meta->getNumOperands(); ++i)
    {
        llvm::MDNode *node  = kern_meta->getOperand(i);
        llvm::Value  *value = node->getOperand(0);
        if (!llvm::isa<llvm::Function>(value)) continue;

        llvm::Function *f = llvm::cast<llvm::Function>(value);
        string s = f->getName().str();
        api_s.push_back(s);
        api.push_back(s.c_str());
    }

    // determine if module has barrier() function calls
    bool hasBarrier = false;
    llvm::CallInst* call;
    for (Module::iterator F = module->begin(),
               EF = module->end(); !hasBarrier && F != EF; ++F)
        for (llvm::inst_iterator I = inst_begin(*F),
                                 E = inst_end(*F); I != E; ++I)
        {
            if (!(call = llvm::dyn_cast<llvm::CallInst>(&*I))) continue;
            if (!call->getCalledFunction())                    continue;
            string name(call->getCalledFunction()->getName());
            if (name == "barrier")
            {
                hasBarrier = true;
                break;
            }
        }

    // Optimize code
    llvm::PassManager *manager = new llvm::PassManager();

    // Common passes (primary goal : remove unused stdlib functions)
    manager->add(llvm::createTypeBasedAliasAnalysisPass());
    manager->add(llvm::createBasicAliasAnalysisPass());

    /*-------------------------------------------------------------------------
    * Do not run this for lib mode as it will result in all functions
    * being removed if main not found.
    *------------------------------------------------------------------------*/
    if (!opt_lib) manager->add(llvm::createInternalizePass(api));

    manager->add(llvm::createIPSCCPPass());
    manager->add(llvm::createGlobalOptimizerPass());
    manager->add(llvm::createConstantMergePass());
    manager->add(llvm::createAlwaysInlinerPass());

    // pocl barrier transformation
    if (hasBarrier)
    {
        manager->add(new llvm::DominatorTree());
        manager->add(new pocl::WorkitemHandlerChooser());
        manager->add(new       BreakConstantGEPs());   // from pocl
        //       add(new       GenerateHeader());      // no need
        manager->add(new pocl::Flatten());
        manager->add(    llvm::createAlwaysInlinerPass());
        manager->add(    llvm::createGlobalDCEPass());
        manager->add(    llvm::createCFGSimplificationPass());
        manager->add(    llvm::createLoopSimplifyPass());
        manager->add(new pocl::PHIsToAllocas());
        manager->add(    llvm::createRegionInfoPass());
        manager->add(new pocl::IsolateRegions());
        manager->add(new pocl::VariableUniformityAnalysis()); // TODO
        manager->add(new pocl::ImplicitLoopBarriers());
        manager->add(new pocl::LoopBarriers());
        manager->add(new pocl::BarrierTailReplication());
        manager->add(new pocl::CanonicalizeBarriers());
        manager->add(new pocl::IsolateRegions());
        manager->add(new pocl::WorkItemAliasAnalysis());
        //       add(new pocl::WorkitemReplication()); // no need
        manager->add(new pocl::WorkitemLoops());
        manager->add(new pocl::AllocasToEntry());
        //       add(new pocl::Workgroup());           // no need
        manager->add(new pocl::TargetAddressSpaces());
    }

    if (optimize)
    {
        /*---------------------------------------------------------------------
        * Inspired by code from "The LLVM Compiler Infrastructure"
        *--------------------------------------------------------------------*/
        manager->add(llvm::createDeadArgEliminationPass());
        manager->add(llvm::createInstructionCombiningPass());
        manager->add(llvm::createFunctionInliningPass());
        manager->add(llvm::createPruneEHPass());   // Remove dead EH info.
        manager->add(llvm::createGlobalOptimizerPass());
        manager->add(llvm::createGlobalDCEPass()); // Remove dead functions.
        manager->add(llvm::createArgumentPromotionPass());
        manager->add(llvm::createInstructionCombiningPass());
        manager->add(llvm::createJumpThreadingPass());

        //ASW TODO maybe turn off re: pete.  might gen bad xlator input
        //manager->add(llvm::createScalarReplAggregatesPass());

        manager->add(llvm::createFunctionAttrsPass()); // Add nocapture.
        manager->add(llvm::createGlobalsModRefPass()); // IP alias analysis.
        manager->add(llvm::createLICMPass());      // Hoist loop invariants.
        manager->add(llvm::createGVNPass());       // Remove redundancies.
        manager->add(llvm::createMemCpyOptPass()); // Remove dead memcpys.
        manager->add(llvm::createDeadStoreEliminationPass());
        manager->add(llvm::createInstructionCombiningPass());
        manager->add(llvm::createJumpThreadingPass());
        manager->add(llvm::createCFGSimplificationPass());
    }

    /*-------------------------------------------------------------------------
    * Builtins will not have workitem functions and do not need wga
    *------------------------------------------------------------------------*/
    if (!opt_builtin) 
    {
        manager->add(llvm::createUnifyFunctionExitNodesPass());
        manager->add(llvm::createTIOpenclWorkGroupAggregationPass(hasBarrier));

        /*---------------------------------------------------------------------
        * Borrow the pocl alloca hoister for the TI simplistic WGA pass as well
        *--------------------------------------------------------------------*/
        if (!hasBarrier)
            manager->add(new pocl::AllocasToEntry());
    }

    manager->add(llvm::createGlobalDCEPass());
    manager->run(*module);
    delete manager;

    return true;
}


/******************************************************************************
* cl6x
******************************************************************************/
bool cl6x(string& filename, string &binary_str)
{
    string bc_file_full(fs_replace_extension(filename, ".bc"));
    string bc_file     (fs_filename(bc_file_full));

    run_cl6x(bc_file, &binary_str, files_other);

    /*-------------------------------------------------------------------------
    * Clean up temporary files 
    *------------------------------------------------------------------------*/
    struct stat statbuf;
    if (!opt_keep)
    {
        const char *name = bc_file_full.c_str();

        unlink(name);

        if (!opt_lib)
        {
            name = fs_replace_extension(bc_file_full, ".obj").c_str();
            unlink(name);
        }

        string bitasm_name(fs_stem(bc_file_full));
        bitasm_name += "_bc.asm";
        name = bitasm_name.c_str();

        unlink(name);

        bitasm_name =fs_stem(bc_file_full);
        bitasm_name += "_bc.obj";
        name = bitasm_name.c_str();

        unlink(name);
    }

    return true;
}

/******************************************************************************
* write_text
******************************************************************************/
void write_text(string filename)
{
    filename = fs_filename(filename);
    string outfile(fs_replace_extension(filename, ".out"));
    string hfile  (fs_replace_extension(filename, ".dsp_h"));

    stringstream bufss;
    bufss << ifstream(outfile.c_str()).rdbuf();

    string buf(bufss.str());

    ofstream header(hfile.c_str(), ios::out);
    
    header << "unsigned int " << fs_stem(filename)
           << "_dsp_bin_len = " << buf.length() << ";" 
           << endl;

    header << "char " << fs_stem(filename) << "_dsp_bin[] = { ";

    int val = buf[0] & 0xff;
    header << "0x"<< hex << setfill('0') << setw(2) << nouppercase <<val<<endl;

    for (int i = 1; i < buf.length(); i++)
    {
        val = buf[i] & 0xff;
        header << ", 0x"<< hex << setfill('0') << setw(2) << nouppercase <<val;
        if (i % 13 == 0) header << endl; 
    }

    header << endl << "};" << endl;
    header.close();
}


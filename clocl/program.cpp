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
#include <llvm/PassManager.h>
#include <llvm/Analysis/Passes.h>
#include <llvm/Analysis/Verifier.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/IPO.h>
#include <llvm/Transforms/Utils/UnifyFunctionExitNodes.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include "wga.h"
#include "file_manip.h"
#include "options.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <unistd.h>

#include <elf.h>

using namespace std;

/******************************************************************************
* Find the C6000 CGT installation
******************************************************************************/
char *get_cgt_install()
{
    char *install = getenv("TI_OCL_CGT_INSTALL");
    if (!install)
    {
        std::cout <<
          "The environment variable TI_OCL_CGT_INSTALL must be set to a "
          << std::endl <<
          "directory path where the C6000 compiler tools are installed. "
          << std::endl;

        abort();
    }

    return install;
}

/******************************************************************************
* Find the OpenCL installation
******************************************************************************/
char *get_ocl_install()
{
    char *install = getenv("TI_OCL_INSTALL");
    if (!install)
    {
        std::cout <<
          "The environment variable TI_OCL_INSTALL must be set to a "
          << std::endl <<
          "directory path where the TI OpenCL product is installed. "
          << std::endl;

        abort();
    }

    return install;
}

std::string get_ocl_dsp()
{
    const char *stdpath = "/usr/share/ti/opencl/dsp";

    struct stat st;
    stat(stdpath, &st);
    if (S_ISDIR(st.st_mode)) return stdpath;

    std::string sinstall = string(get_ocl_install()) + "/dsp";
    return sinstall;
}

/******************************************************************************
* run_cl6x
******************************************************************************/
int run_cl6x(string filename, string *llvm_bitcode, string addl_files)
{
    string command("cl6x --f -q --abi=eabi --use_g3 -mv6600 -mt -mo ");

    if (opt_keep) command += "-mw -k --z ";

    command += "--disable:sploop ";

    if (opt_debug) command += "-g -o0 ";
    else           command += "-o3 ";

    const char *cgt_install = get_cgt_install();

   command += "-I"; command += cgt_install; command += "/include ";
   command += "-I"; command += cgt_install; command += "/lib ";
   command += "-I"; command += get_ocl_dsp().c_str(); command += " ";

    command += "--bc_file="; command += filename; command += " "; 

    /*-------------------------------------------------------------------------
    * Encode LLVM bitcode as bytes in the .llvmir section of the .asm file
    *------------------------------------------------------------------------*/
    if (llvm_bitcode != NULL)
    {
        string bitasm_name(fs_stem(filename));
        bitasm_name += "_bc.asm";

        ofstream outasmfile(bitasm_name.c_str(), ios::out);
        outasmfile << "\t.sect \".llvmir\"\n" << "\t.retain";
        int nbytes = llvm_bitcode->size();
        for (int i = 0; i < nbytes; i++)
            if (i % 10 == 0)
               outasmfile << "\n\t.byte " << (int) llvm_bitcode->at(i);
            else
               outasmfile << ", " << (int) llvm_bitcode->at(i);
        outasmfile.close();

        command += bitasm_name; command += " ";
    }

    if (opt_lib)
    {
        if (opt_verbose) cout << command << endl;
        int x = system(command.c_str());
        return true;
    }

    string outfile(fs_replace_extension(filename, ".out"));

    command += "-z ";
    command += "-o "; 
    command += outfile;
    command += " "; 

    if (opt_keep) 
    { 
        command += "-m "; 
        command += fs_replace_extension(filename, ".map");
        command += " "; 
    }

    /*-------------------------------------------------------------------------
    * Any libraries or object files need to go last to resolve references
    *------------------------------------------------------------------------*/
    command += addl_files; 
    command += " -ldsp.syms ";

    if (opt_verbose) cout << command << endl;
    int x = system(command.c_str());

    if (!opt_debug)
    {
        string strip_command("strip6x ");
        strip_command += outfile;
        if (opt_verbose) cout << strip_command << endl;
        x = system(strip_command.c_str());
    }
}

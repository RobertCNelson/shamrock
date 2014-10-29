; ModuleID = 'vector_args.bc'
target datalayout = "e-p:32:32-i64:64-v16:16-v24:32-v32:32-v48:64-v96:128-v192:256-v256:256-v512:512-v1024:1024"
target triple = "spir-unknown-unknown-unkown"

; Function Attrs: nounwind readnone
define void @test_kernel(i8 signext %c, i8 zeroext %uc, i16 signext %s, i16 zeroext %us, i32 %i, i32 %ui, float %f, float addrspace(1)* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone
define void @test_kernel2(<2 x i8> %c, <2 x i8> %uc, <2 x i16> %s, <2 x i16> %us, <2 x i32> %i, <2 x i32> %ui, <2 x float> %f, <2 x float> addrspace(1)* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone
define void @test_kernel4(<4 x i8> %c, <4 x i8> %uc, <4 x i16> %s, <4 x i16> %us, <4 x i32> %i, <4 x i32> %ui, <4 x float> %f, <4 x float> addrspace(1)* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone
define void @test_kernel8(<8 x i8> %c, <8 x i8> %uc, <8 x i16> %s, <8 x i16> %us, <8 x i32> %i, <8 x i32> %ui, <8 x float> %f, <8 x float> addrspace(1)* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone
define void @test_kernel16(<16 x i8> %c, <16 x i8> %uc, <16 x i16> %s, <16 x i16> %us, <16 x i32> %i, <16 x i32> %ui, <16 x float> %f, <16 x float> addrspace(1)* nocapture %result) #0 {
entry:
  ret void
}

attributes #0 = { nounwind readnone "less-precise-fpmad"="false" "no-frame-pointer-elim"="true" "no-frame-pointer-elim-non-leaf" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }

!opencl.kernels = !{!0, !1, !2, !3, !4}
!llvm.ident = !{!5}

!0 = metadata !{void (i8, i8, i16, i16, i32, i32, float, float addrspace(1)*)* @test_kernel}
!1 = metadata !{void (<2 x i8>, <2 x i8>, <2 x i16>, <2 x i16>, <2 x i32>, <2 x i32>, <2 x float>, <2 x float> addrspace(1)*)* @test_kernel2}
!2 = metadata !{void (<4 x i8>, <4 x i8>, <4 x i16>, <4 x i16>, <4 x i32>, <4 x i32>, <4 x float>, <4 x float> addrspace(1)*)* @test_kernel4}
!3 = metadata !{void (<8 x i8>, <8 x i8>, <8 x i16>, <8 x i16>, <8 x i32>, <8 x i32>, <8 x float>, <8 x float> addrspace(1)*)* @test_kernel8}
!4 = metadata !{void (<16 x i8>, <16 x i8>, <16 x i16>, <16 x i16>, <16 x i32>, <16 x i32>, <16 x float>, <16 x float> addrspace(1)*)* @test_kernel16}
!5 = metadata !{metadata !"clang version 3.6.0 (http://llvm.org/git/clang.git 01adae8f440672196da28be6fce2bb4acf8ab40b) (http://llvm.org/git/llvm.git f7be7f15c1ff2882719f823fbe270e48bb0f4340)"}

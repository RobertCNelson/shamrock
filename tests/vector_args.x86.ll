; ModuleID = 'vector_args.bc'
target datalayout = "e-m:e-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

; Function Attrs: nounwind readnone uwtable
define void @test_kernel(i8 signext %c, i8 zeroext %uc, i16 signext %s, i16 zeroext %us, i32 %i, i32 %ui, float %f, float* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone uwtable
define void @test_kernel2(<2 x i8>* byval nocapture align 8, <2 x i8>* byval nocapture align 8, i32 %s.coerce, i32 %us.coerce, double %i.coerce, double %ui.coerce, double %f.coerce, <2 x float>* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone uwtable
define void @test_kernel4(i32 %c.coerce, i32 %uc.coerce, double %s.coerce, double %us.coerce, <4 x i32> %i, <4 x i32> %ui, <4 x float> %f, <4 x float>* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone uwtable
define void @test_kernel8(double %c.coerce, double %uc.coerce, <8 x i16> %s, <8 x i16> %us, <8 x i32>* byval nocapture align 32, <8 x i32>* byval nocapture align 32, <8 x float>* byval nocapture align 32, <8 x float>* nocapture %result) #0 {
entry:
  ret void
}

; Function Attrs: nounwind readnone uwtable
define void @test_kernel16(<16 x i8> %c, <16 x i8> %uc, <16 x i16>* byval nocapture align 32, <16 x i16>* byval nocapture align 32, <16 x i32>* byval nocapture align 64, <16 x i32>* byval nocapture align 64, <16 x float>* byval nocapture align 64, <16 x float>* nocapture %result) #0 {
entry:
  ret void
}

attributes #0 = { nounwind readnone uwtable "less-precise-fpmad"="false" "no-frame-pointer-elim"="false" "no-infs-fp-math"="false" "no-nans-fp-math"="false" "stack-protector-buffer-size"="8" "unsafe-fp-math"="false" "use-soft-float"="false" }

!opencl.kernels = !{!0, !1, !2, !3, !4}
!llvm.ident = !{!5}

!0 = metadata !{void (i8, i8, i16, i16, i32, i32, float, float*)* @test_kernel}
!1 = metadata !{void (<2 x i8>*, <2 x i8>*, i32, i32, double, double, double, <2 x float>*)* @test_kernel2}
!2 = metadata !{void (i32, i32, double, double, <4 x i32>, <4 x i32>, <4 x float>, <4 x float>*)* @test_kernel4}
!3 = metadata !{void (double, double, <8 x i16>, <8 x i16>, <8 x i32>*, <8 x i32>*, <8 x float>*, <8 x float>*)* @test_kernel8}
!4 = metadata !{void (<16 x i8>, <16 x i8>, <16 x i16>*, <16 x i16>*, <16 x i32>*, <16 x i32>*, <16 x float>*, <16 x float>*)* @test_kernel16}
!5 = metadata !{metadata !"clang version 3.6.0 (http://llvm.org/git/clang.git 01adae8f440672196da28be6fce2bb4acf8ab40b) (http://llvm.org/git/llvm.git f7be7f15c1ff2882719f823fbe270e48bb0f4340)"}

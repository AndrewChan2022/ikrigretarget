//
//  SoulRetargetSkeleton.h
//
//
//  Created by kai chen on 3/24/23.
//

#pragma once

#include <vector>
#include <unordered_map>
#include <string>
#include <memory>
#include <algorithm>
#include <cmath>

#include "glm/vec3.hpp" // glm::vec3
#include "glm/vec4.hpp" // glm::vec4
#include "glm/gtc/quaternion.hpp"   // glm::quat
#include "glm/mat4x4.hpp" // glm::mat4
#include "glm/ext/matrix_transform.hpp" // glm::translate, glm::rotate, glm::scale
#include "glm/ext/matrix_clip_space.hpp" // glm::perspective
#include "glm/ext/scalar_constants.hpp" // glm::pi
#include "glm/gtx/matrix_decompose.hpp"


namespace SoulIK
{

    using FName = std::string;
    #define TArray std::vector
    using int32 = int32_t;
    #define TMap std::unordered_map
    #define INDEX_NONE  -1
    #define NAME_None ""
    #define KINDA_SMALL_NUMBER  (1.e-4f)
    #define UE_SMALL_NUMBER     (1.e-8f)

    //static constexpr double ZeroTolerance = 1e-08;
    #define ZeroTolerance (1e-08)
    #define UE_THRESH_QUAT_NORMALIZED  (0.01f)

    #define checkNoEntry() printf("error, get into default")

    static inline bool IsNearlyEqual(float A, float B, float ErrorTolerance = UE_SMALL_NUMBER)
	{
		return std::abs( A - B ) <= ErrorTolerance;
	}

    namespace FText {
        const char* FromName(std::string s);
    };

    struct FVector;
    FVector operator-(double Scale, const FVector& V);
    struct FVector : public glm::dvec3 {
        FVector(const glm::dvec3& v) : FVector(v.x, v.y, v.z) {}
        FVector():FVector(0, 0, 0) {}
        FVector(double _x, double _y, double _z): glm::dvec3(_x, _y, _z){}
        FVector(double _x): glm::dvec3(_x) {}
        static const FVector ZeroVector;
        static const FVector OneVector;

        // Unit X axis vector (1,0,0)
        static const FVector XAxisVector;
	    // Unit Y axis vector (0,1,0) 
	    static const FVector YAxisVector;
	    // Unit Z axis vector (0,0,1)
	    static const FVector ZAxisVector;

        void Set(double InX, double InY, double InZ)
        {
            x = InX;
            y = InY;
            z = InZ;
        }


        FVector operator+(const FVector& V) const {
            return FVector(x + V.x, y + V.y, z + V.z);
        }
        FVector operator-(const FVector& V) const {
            return FVector(x - V.x, y - V.y, z - V.z);
        }
        FVector operator*(const double s) const {
            return FVector(x * s, y * s, z * s);
        }
        FVector operator*(const FVector& V) const {
            return FVector(x * V.x, y * V.y, z * V.z);
        }
        FVector operator/(const double s) const {
            return FVector(x / s, y / s, z / s);
        }
        FVector operator+=(const FVector& V)
        {
            x += V.x; y += V.y; z += V.z;
            return *this;
        }
        FVector operator-=(const FVector& V)
        {
            x -= V.x; y -= V.y; z -= V.z;
            return *this;
        }
        FVector operator*=(const double& s)
        {
            x *= s; y *= s; z *= s;
            return *this;
        }
        FVector operator*=(const FVector& V) {
            x *= V.x; y *= V.y; z *= V.z;
            return *this;
        }
        FVector operator/=(const double& s)
        {
            x /= s; y /= s; z /= s;
            return *this;
        }



        double Size() const
        {
            return std::sqrt(x*x + y*y + z*z);
        }

        double Length() const
        {
            return Size();
        }

        double SizeSquared() const
        {
            return x*x + y*y + z*z;
        }
        double SquaredLength() const
        {
            return SizeSquared();
        }
        static FVector CrossProduct(const FVector& A, const FVector& B)
        {
            glm::dvec3 C = glm::cross(A, B);
            return FVector(C);
        }
        static FVector lerp(const FVector& A, const FVector& B, const double Alpha) {
            return A * (1.0 - Alpha) + B * Alpha;
        }
        static FVector lerp(const FVector& A, const FVector& B, const FVector& Alpha) {
            return A * (1.0 - Alpha) + B * Alpha;
        }
    };

    FVector operator*(double Scale, const FVector& V);
    FVector operator-(double Scale, const FVector& V);

    struct FQuat;
    struct FRotator : public glm::dvec3 {
        // pitch yaw roll == y z x
        FRotator():FRotator(0, 0, 0){}
        FRotator(double InPitch, double InYaw, double InRoll ) : glm::dvec3(InPitch, InYaw, InRoll) {}
        explicit FRotator(const FQuat& q);
        FQuat Quaternion();

        static const FRotator ZeroRotator;
    };

    // glm::quat store wxyz 
    // FQuat store xyzw
    struct FQuat : public glm::dquat {
        // glm:  w, x, y, z
        // q.xyz = axis.xyz * sin(angle / 2f);
        // q.w = cos(angle / 2f);
        // glm::quat q(q.w, q.x, q.y, q.z);  // w,x,y,z
        // glm::quat q = glm::quat(glm::vec3(pitch, yaw, roll));
        // glm::quat q = glm::angleAxis( glm::radians(45.f), glm::vec3( 0.707f, 0.707f, 0. ) );
        // 
        // FQuat: x, y, z, w
        // fq.xyz = axis.xyz * sin(angle / 2f);
        // fq.w = cos(angle / 2f);
        // FQuat q(q2.x, q2.y, q2.z, q2.w);
        FQuat(glm::dquat q) : FQuat(q.x, q.y, q.z, q.w){}
        FQuat() : FQuat(0, 0, 0, 1){}
        FQuat(double _x, double _y, double _z, double _w) : glm::dquat(_w, _x, _y, _z) {}
        explicit FQuat(const FRotator& rotator) {
            glm::dquat q = glm::dquat(glm::dvec3(rotator.x, rotator.y, rotator.z));
            w = q.w;
            x = q.x;
            y = q.y;
            z = q.z;
        }
        FQuat(const FQuat& other) {
            w = other.w;
            x = other.x;
            y = other.y;
            z = other.z;
        }

        static const FQuat Identity;
        FRotator Rotator() const
	    {
            glm::dquat q2(w, x, y, z);  // w,x,y,z
            glm::dvec3 euler = glm::eulerAngles(q2);
            return FRotator(euler.x, euler.y, euler.z);
	    }

        FQuat Inverse() const {
            return FQuat(-x, -y, -z, w);
        }
        bool IsNormalized() const {
            return (std::abs(1.f - SizeSquared()) < UE_THRESH_QUAT_NORMALIZED);
        }
        double Size() const
        {
            return std::sqrt(x * x + y * y + z * z + w * w);
        }
        double SizeSquared() const
        {
	        return (x * x + y * y + z * z + w * w);
        }
        double getAngle() const
        {
            return std::acos(w) * 2.0 ;
        }
        double getAngleDegree() const
        {
            return std::acos(w) * 2.0 * 180.0 / 3.1415926;
        }

        FQuat GetNormalized(double Tolerance = UE_SMALL_NUMBER) const
        {
            FQuat Result(*this);
            Result.Normalize(Tolerance);
            return Result;
        }
        void Normalize(double Tolerance = UE_SMALL_NUMBER)
        {
            const double SquareSum = x * x + y * y + z * z + w * w;

            if (SquareSum >= Tolerance)
            {
                const double Scale = 1.0 / std::sqrt(SquareSum);

                x *= Scale; 
                x *= Scale; 
                z *= Scale;
                w *= Scale;
            }
            else
            {
                *this = Identity;
            }
        }

        FVector RotateVector(FVector V) const
        {	
            // http://people.csail.mit.edu/bkph/articles/Quaternions.pdf
            // V' = V + 2w(Q x V) + (2Q x (Q x V))
            // refactor:
            // V' = V + w(2(Q x V)) + (Q x (2(Q x V)))
            // T = 2(Q x V);
            // V' = V + w*(T) + (Q x T)

            const FVector Q(x, y, z);
            const FVector TT = 2.f * FVector::CrossProduct(Q, V);
            const FVector Result = V + (w * TT) + FVector::CrossProduct(Q, TT);
            return Result;
        }

        double operator|(const FQuat& Q) const
        {
            return x * Q.x + y * Q.y + z * Q.z + w * Q.w;
        }
        void VectorQuaternionMultiply(FQuat* Result, const FQuat* Quat1, const FQuat* Quat2) const {
            // input wxyz, but need store:  xyzw
            typedef double Double4[4];
            const double A[4] =  {Quat1->x, Quat1->y, Quat1->z, Quat1->w};
            const double B[4] =  {Quat2->x, Quat2->y, Quat2->z, Quat2->w};
            double R[4] =  {Quat2->x, Quat2->y, Quat2->z, Quat2->w};
            
            //const Double4& A = *((const Double4*)Quat1);
            //const Double4& B = *((const Double4*)Quat2);
            //Double4& R = *((Double4*)Result);
        #define USE_FAST_QUAT_MUL 1
        #if USE_FAST_QUAT_MUL
            const double T0 = (A[2] - A[1]) * (B[1] - B[2]);
            const double T1 = (A[3] + A[0]) * (B[3] + B[0]);
            const double T2 = (A[3] - A[0]) * (B[1] + B[2]);
            const double T3 = (A[1] + A[2]) * (B[3] - B[0]);
            const double T4 = (A[2] - A[0]) * (B[0] - B[1]);
            const double T5 = (A[2] + A[0]) * (B[0] + B[1]);
            const double T6 = (A[3] + A[1]) * (B[3] - B[2]);
            const double T7 = (A[3] - A[1]) * (B[3] + B[2]);
            const double T8 = T5 + T6 + T7;
            const double T9 = 0.5 * (T4 + T8);

            R[0] = T1 + T9 - T8;
            R[1] = T2 + T9 - T7;
            R[2] = T3 + T9 - T6;
            R[3] = T0 + T9 - T5;
        #else
            // store intermediate results in temporaries
            const double TX = A[3] * B[0] + A[0] * B[3] + A[1] * B[2] - A[2] * B[1];
            const double TY = A[3] * B[1] - A[0] * B[2] + A[1] * B[3] + A[2] * B[0];
            const double TZ = A[3] * B[2] + A[0] * B[1] - A[1] * B[0] + A[2] * B[3];
            const double TW = A[3] * B[3] - A[0] * B[0] - A[1] * B[1] - A[2] * B[2];

            // copy intermediate result to *this
            R[0] = TX;
            R[1] = TY;
            R[2] = TZ;
            R[3] = TW;
        #endif

            Result->w = R[3];
            Result->x = R[0];
            Result->y = R[1];
            Result->z = R[2];
        }
        // use parent multiply
        //FQuat operator*(const FQuat& Q) const {
	        //FQuat Result;
            //VectorQuaternionMultiply(&Result, this, &Q);
            //return Result;
        //}
        
        static float FloatSelect(const float Comparand, const float ValueGEZero, const float ValueLTZero )
        {
            return Comparand >= 0.f ? ValueGEZero : ValueLTZero;
        }

        static FQuat FastLerp(const FQuat& A, const FQuat& B, const double Alpha)
        {
            // To ensure the 'shortest route', we make sure the dot product between the both rotations is positive.
            const double DotResult = (A | B);
            const double Bias = FloatSelect(DotResult, double(1.0f), double(-1.0f));
            return (B * Alpha) + (A * (Bias * (1.f - Alpha)));
        }
    };

    struct FTransform
    {
        // Rotation of this transformation, as a quaternion.
        FQuat   Rotation;
	    // Translation of this transformation, as a vector.
	    FVector Translation;
	    // 3D scale (always applied in local space) as a vector.
	    FVector  Scale3D;

        static const FTransform Identity;

        FTransform()
		: Rotation(0.f, 0.f, 0.f, 1.f)
		, Translation(0.f)
		, Scale3D(FVector::OneVector)
	    {
	    }

        explicit FTransform(const FVector& InTranslation)
		: Rotation(FQuat::Identity),
		Translation(InTranslation),
		Scale3D(FVector::OneVector) 
        {
        }

        explicit FTransform(const FQuat& InRotation)
		: Rotation(InRotation),
		Translation(FVector::ZeroVector),
		Scale3D(FVector::OneVector)
	    {
        }

        explicit FTransform(const FRotator& InRotation)
		: Rotation(InRotation),
		Translation(FVector::ZeroVector),
		Scale3D(FVector::OneVector)
	    {
        }

        FTransform(const FQuat& InRotation, const FVector& InTranslation, const FVector& InScale3D = FVector::OneVector)
		: Rotation(InRotation),
		Translation(InTranslation),
		Scale3D(InScale3D)
	    {
        }

        FTransform(const FRotator& InRotation, const FVector& InTranslation, const FVector& InScale3D = FVector::OneVector)
		: Rotation(InRotation),
		Translation(InTranslation),
		Scale3D(InScale3D)
	    {
        }
        explicit FTransform(const glm::mat4 InMatrix)
	    {
		    SetFromMatrix(InMatrix);
        }
        explicit FTransform(const glm::dmat4 InMatrix)
	    {
		    SetFromMatrix(InMatrix);
        }

        // Constructor that takes basis axes and translation
	    FTransform(const FVector& InX, const FVector& InY, const FVector& InZ, const FVector& InTranslation)
	    {
            glm::dvec4 X(InX.x, InX.y, InX.z, 0);
            glm::dvec4 Y(InY.x, InY.y, InY.z, 0);
            glm::dvec4 Z(InZ.x, InZ.y, InZ.z, 0);
            glm::dvec4 T(InTranslation.x, InTranslation.y, InTranslation.z, 1);
            glm::dmat4 m(X, Y, Z, T);

		    SetFromMatrix(m);
	    }

        void SetFromMatrix(const glm::dmat4& m) {
            glm::dvec3 scale;
            glm::dvec3 translation;
            glm::dvec3 skew;
            glm::dvec4 perspective;
            glm::dquat q;

            glm::decompose(m, scale, q, translation, skew, perspective);
            Rotation = FQuat(q.x, q.y, q.z, q.w);
            Scale3D = FVector(scale.x, scale.y, scale.z);
            Translation = FVector(translation.x, translation.y, translation.z);
        }
        void SetFromMatrix(const glm::mat4& m) {
            glm::vec3 scale;
            glm::vec3 translation;
            glm::vec3 skew;
            glm::vec4 perspective;
            glm::quat q;

            glm::decompose(m, scale, q, translation, skew, perspective);
            Rotation = FQuat(q.x, q.y, q.z, q.w);
            Scale3D = FVector(scale.x, scale.y, scale.z);
            Translation = FVector(translation.x, translation.y, translation.z);
        }
        
        FTransform Inverse() const
        {
            FQuat   InvRotation = Rotation.Inverse();
            FVector InvScale3D = GetSafeScaleReciprocal(Scale3D);

            glm::dquat invq(InvRotation.w, InvRotation.x, InvRotation.y, InvRotation.z);
            glm::dvec3 invs(InvScale3D.x, InvScale3D.y, InvScale3D.z);
            glm::dvec3 t(Translation.x, Translation.y, Translation.z);
            glm::dvec3 invt = invq * (invs * (-t));

            return FTransform(FQuat(invq), FVector(invt), FVector(invs));
        }

        FTransform operator*(double Mult) const
        {
            return FTransform(Rotation * Mult, Translation * Mult, Scale3D * Mult);
        }

        FTransform& operator*=(double Mult)
        {
            Translation *= Mult;
            Rotation.x *= Mult;
            Rotation.y *= Mult;
            Rotation.z *= Mult;
            Rotation.w *= Mult;
            Scale3D *= Mult;

            return *this;
        }

        void Multiply(FTransform* OutTransform, const FTransform* A, const FTransform* B) const
        {

            //	When Q = quaternion, S = single scalar scale, and T = translation
            //	QST(A) = Q(A), S(A), T(A), and QST(B) = Q(B), S(B), T(B)

            //	QST (AxB) 

            // QST(A) = Q(A)*S(A)*P*-Q(A) + T(A)
            // QST(AxB) = Q(B)*S(B)*QST(A)*-Q(B) + T(B)
            // QST(AxB) = Q(B)*S(B)*[Q(A)*S(A)*P*-Q(A) + T(A)]*-Q(B) + T(B)
            // QST(AxB) = Q(B)*S(B)*Q(A)*S(A)*P*-Q(A)*-Q(B) + Q(B)*S(B)*T(A)*-Q(B) + T(B)
            // QST(AxB) = [Q(B)*Q(A)]*[S(B)*S(A)]*P*-[Q(B)*Q(A)] + Q(B)*S(B)*T(A)*-Q(B) + T(B)

            //	Q(AxB) = Q(B)*Q(A)
            //	S(AxB) = S(A)*S(B)
            //	T(AxB) = Q(B)*S(B)*T(A)*-Q(B) + T(B)

            if (AnyHasNegativeScale(A->Scale3D, B->Scale3D))
            {
                // @note, if you have 0 scale with negative, you're going to lose rotation as it can't convert back to quat
                //MultiplyUsingMatrixWithScale(OutTransform, A, B);
                glm::dmat4 m1 = A->ToMatrixWithScale();
                glm::dmat4 m2 = B->ToMatrixWithScale();
                glm::dmat4 m3 = m1 * m2;
                *OutTransform = FTransform(m3);
            }
            else
            {
                OutTransform->Rotation = B->Rotation*A->Rotation;
                OutTransform->Scale3D = A->Scale3D*B->Scale3D;
                OutTransform->Translation = B->Rotation*(B->Scale3D*A->Translation) + B->Translation;
            }

            // we do not support matrix transform when non-uniform
            // that was removed at rev 21 with UE4
        }

        FTransform operator*(const FTransform& Other) const
        {
            FTransform Output;
            Multiply(&Output, this, &Other);
            return Output;
        }
        void operator*=(const FTransform& Other)
        {
	        Multiply(this, this, &Other);
        }
        FTransform operator/(const FTransform& Other) const
        {
            FTransform Output;
            Output = GetRelativeTransform(Other);
            return Output;
        }
        //FTransform operator*(const FQuat& Other) const;
        //void operator*=(const FQuat& Other);

        FVector GetSafeScaleReciprocal(const FVector& InScale, double Tolerance = ZeroTolerance) const
        {
            FVector SafeReciprocalScale;
            if (std::abs(InScale.x) <= Tolerance)
            {
                SafeReciprocalScale.x = 0.f;
            }
            else
            {
                SafeReciprocalScale.x = 1 / InScale.x;
            }

            if (std::abs(InScale.y) <= Tolerance)
            {
                SafeReciprocalScale.y = 0.f;
            }
            else
            {
                SafeReciprocalScale.y = 1 / InScale.y;
            }

            if (std::abs(InScale.z) <= Tolerance)
            {
                SafeReciprocalScale.z = 0.f;
            }
            else
            {
                SafeReciprocalScale.z = 1 / InScale.z;
            }

            return SafeReciprocalScale;
        }

        void AddToTranslation(const FVector& DeltaTranslation)
        {
            Translation += DeltaTranslation;
        
        }

        FQuat GetRotation() const
        {
            return Rotation;
        }
        void SetRotation(const FQuat& NewRotation)
        {
            Rotation = NewRotation;
        }
        FVector GetTranslation() const
        {
            return Translation;
        }
        void SetTranslation(const FVector& NewTranslation)
	    {
		    Translation = NewTranslation;
	    }
        FVector GetScale3D() const
        {
            return Scale3D;
        }
        void SetScale3D(const FVector& NewScale3D)
        {
            Scale3D = NewScale3D;
        }
        
        
        FTransform GetRelativeTransform(const FTransform& Other) const {
            // A * B(-1) = VQS(B)(-1) (VQS (A))
            // 
            // Scale = S(A)/S(B)
            // Rotation = Q(B)(-1) * Q(A)
            // Translation = 1/S(B) *[Q(B)(-1)*(T(A)-T(B))*Q(B)]
            // where A = this, B = Other
            FTransform Result;

            if (AnyHasNegativeScale(Scale3D, Other.GetScale3D()))
            {
                // @note, if you have 0 scale with negative, you're going to lose rotation as it can't convert back to quat
                GetRelativeTransformUsingMatrixWithScale(&Result, this, &Other);
            }
            else
            {
                FVector SafeRecipScale3D = GetSafeScaleReciprocal(Other.Scale3D, UE_SMALL_NUMBER);
                Result.Scale3D = Scale3D*SafeRecipScale3D;

                if (Other.Rotation.IsNormalized() == false)
                {
                    return FTransform::Identity;
                }

                FQuat Inverse = Other.Rotation.Inverse();
                Result.Rotation = Inverse*Rotation;

                Result.Translation = (Inverse*(Translation - Other.Translation))*(SafeRecipScale3D);
            }

            return Result;
        }
        bool AnyHasNegativeScale(const FVector& InScale3D, const  FVector& InOtherScale3D) const
        {
            return  (InScale3D.x < 0.f || InScale3D.y < 0.f || InScale3D.z < 0.f 
            || InOtherScale3D.x < 0.f || InOtherScale3D.y < 0.f || InOtherScale3D.z < 0.f );
        }
        void GetRelativeTransformUsingMatrixWithScale(FTransform* OutTransform, const FTransform* Base, const FTransform* Relative) const
        {

            glm::dmat4 m1 = Base->ToMatrixWithScale();
            glm::dmat4 m2 = Relative->ToMatrixWithScale();
            glm::dmat4 m3 = m1 / m2;
            *OutTransform = FTransform(m3);

            // // the goal of using M is to get the correct orientation
            // // but for translation, we still need scale
            // glm::dmat4 AM = Base->ToMatrixWithScale();
            // glm::dmat4 BM = Relative->ToMatrixWithScale();
            // // get combined scale
            // FVector SafeRecipScale3D = GetSafeScaleReciprocal(Relative->Scale3D, UE_SMALL_NUMBER);
            // FVector DesiredScale3D = Base->Scale3D*SafeRecipScale3D;
            // ConstructTransformFromMatrixWithDesiredScale(AM, glm::inverse(BM), DesiredScale3D, *OutTransform);
        }


        // Convert this Transform to a transformation matrix, ignoring its scaling
        glm::dmat4 ToMatrixNoScale() const
        {
            glm::dmat4 OutMatrix;

            OutMatrix[3][0] = Translation.x;
            OutMatrix[3][1] = Translation.y;
            OutMatrix[3][2] = Translation.z;

            const double x2 = Rotation.x + Rotation.x;
            const double y2 = Rotation.y + Rotation.y;
            const double z2 = Rotation.z + Rotation.z;
            {
                const double xx2 = Rotation.x * x2;
                const double yy2 = Rotation.y * y2;
                const double zz2 = Rotation.z * z2;

                OutMatrix[0][0] = (1.0f - (yy2 + zz2));
                OutMatrix[1][1] = (1.0f - (xx2 + zz2));
                OutMatrix[2][2] = (1.0f - (xx2 + yy2));
            }
            {
                const double yz2 = Rotation.y * z2;
                const double wx2 = Rotation.w * x2;

                OutMatrix[2][1] = (yz2 - wx2);
                OutMatrix[1][2] = (yz2 + wx2);
            }
            {
                const double xy2 = Rotation.x * y2;
                const double wz2 = Rotation.w * z2;

                OutMatrix[1][0] = (xy2 - wz2);
                OutMatrix[0][1] = (xy2 + wz2);
            }
            {
                const double xz2 = Rotation.x * z2;
                const double wy2 = Rotation.w * y2;

                OutMatrix[2][0] = (xz2 + wy2);
                OutMatrix[0][2] = (xz2 - wy2);
            }

            OutMatrix[0][3] = 0.0f;
            OutMatrix[1][3] = 0.0f;
            OutMatrix[2][3] = 0.0f;
            OutMatrix[3][3] = 1.0f;

            return OutMatrix;
        }

        // Convert this Transform to a transformation matrix, with its scaling
        glm::dmat4 ToMatrixWithScale() const
        {
            glm::dmat4 OutMatrix;

            // col major
            OutMatrix[3][0] = Translation.x;
            OutMatrix[3][1] = Translation.y;
            OutMatrix[3][2] = Translation.z;

            const double x2 = Rotation.x + Rotation.x;
            const double y2 = Rotation.y + Rotation.y;
            const double z2 = Rotation.z + Rotation.z;
            {
                const double xx2 = Rotation.x * x2;
                const double yy2 = Rotation.y * y2;
                const double zz2 = Rotation.z * z2;

                OutMatrix[0][0] = (1.0f - (yy2 + zz2)) * Scale3D.x;
                OutMatrix[1][1] = (1.0f - (xx2 + zz2)) * Scale3D.y;
                OutMatrix[2][2] = (1.0f - (xx2 + yy2)) * Scale3D.z;
            }
            {
                const double yz2 = Rotation.y * z2;
                const double wx2 = Rotation.w * x2;

                OutMatrix[2][1] = (yz2 - wx2) * Scale3D.z;
                OutMatrix[1][2] = (yz2 + wx2) * Scale3D.y;
            }
            {
                const double xy2 = Rotation.x * y2;
                const double wz2 = Rotation.w * z2;

                OutMatrix[1][0] = (xy2 - wz2) * Scale3D.y;
                OutMatrix[0][1] = (xy2 + wz2) * Scale3D.x;
            }
            {
                const double xz2 = Rotation.x * z2;
                const double wy2 = Rotation.w * y2;

                OutMatrix[2][0] = (xz2 + wy2) * Scale3D.z;
                OutMatrix[0][2] = (xz2 - wy2) * Scale3D.x;
            }

            OutMatrix[0][3] = 0.0f;
            OutMatrix[1][3] = 0.0f;
            OutMatrix[2][3] = 0.0f;
            OutMatrix[3][3] = 1.0f;

            return OutMatrix;
        }

        FVector TransformPosition(const FVector& V) const
        {
            return Rotation.RotateVector(Scale3D*V) + Translation;
        }
        

        // void ConstructTransformFromMatrixWithDesiredScale(const glm::dmat4& AMatrix, const glm::dmat4& BMatrix, const FVector& DesiredScale, FTransform& OutTransform) const
        // {
        //     // todo

        //     // the goal of using M is to get the correct orientation
        //     // but for translation, we still need scale
        //     glm::dmat4 M = AMatrix * BMatrix;
        //     M.RemoveScaling();

        //     // apply negative scale back to axes
        //     FVector SignedScale = DesiredScale.GetSignVector();

        //     M.SetAxis(0, SignedScale.X * M.GetScaledAxis(EAxis::X));
        //     M.SetAxis(1, SignedScale.Y * M.GetScaledAxis(EAxis::Y));
        //     M.SetAxis(2, SignedScale.Z * M.GetScaledAxis(EAxis::Z));

        //     // @note: if you have negative with 0 scale, this will return rotation that is identity
        //     // since matrix loses that axes
        //     TQuat<T> Rotation = TQuat<T>(M);
        //     Rotation.Normalize();

        //     // set values back to output
        //     OutTransform.Scale3D = DesiredScale;
        //     OutTransform.Rotation = Rotation;

        //     // technically I could calculate this using TTransform<T> but then it does more quat multiplication 
        //     // instead of using Scale in matrix multiplication
        //     // it's a question of between RemoveScaling vs using TTransform<T> to move translation
        //     OutTransform.Translation = M.GetOrigin();
        // }

        //*
        //quat.GetNormalized
        //lerpquat


    };
}
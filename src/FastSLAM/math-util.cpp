/**
 * @file math-util.cpp
 * @brief Implements common probability distributions and their samplers.
 */

#include "math-util.h"

std::pair<float, float> MathUtil::worldToBody2D( const std::pair<float,
                                                                 float>& aWorld_quant,
                                                 const float aTheta_rad ){

   float x_body = cosf(aTheta_rad) * aWorld_quant.first + sinf(aTheta_rad) *
                  aWorld_quant.second;
   float y_body = -sinf(aTheta_rad) * aWorld_quant.first + cosf(aTheta_rad) *
                  aWorld_quant.second;
   return std::make_pair(x_body, y_body);

}

std::pair<float, float> MathUtil::bodyToWorld2D( const std::pair<float,
                                                                 float>& aBody_quant,
                                                 const float aTheta_rad ){

   float x_world = cosf(aTheta_rad) * aBody_quant.first - sinf(aTheta_rad) *
                   aBody_quant.second;
   float y_world = sinf(aTheta_rad) * aBody_quant.first + cosf(aTheta_rad) *
                   aBody_quant.second;
   return std::make_pair(x_world, y_world);

}

float MathUtil::sampleNormal( const float aMean, const float aVariance ){

   if (aVariance < 0.0) return std::numeric_limits<double>::quiet_NaN();

   std::random_device rd{};
   std::mt19937 gen{rd()};
   std::normal_distribution d{aMean, sqrtf(aVariance)};

   return d(gen);
}

float MathUtil::sampleUniform( const float& aMin, const float& aMax ){
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(aMin, aMax);
    return dist(gen);
}

float MathUtil::findDist( const struct Point2D& aPointA, const struct Point2D& aPointB ){
   return sqrtf( powf((aPointA.x - aPointB.x), 2) + powf((aPointA.y - aPointB.y), 2) );
}

float MathUtil::findDist( const struct Point2D& aLandMark, const struct Pose2D& aRobPose ){
   return sqrtf( powf((aLandMark.x - aRobPose.x), 2) + powf((aLandMark.y - aRobPose.y), 2) );
}

template< class T >
T MathUtil::genCDF( const std::vector<T>& aPdfVec, std::vector<T>& aTargetVec ){

    if (aPdfVec.empty()) return static_cast<T>(0);

    T running_sum = static_cast<T>(0);
    aTargetVec.push_back(running_sum); // 0 needs to be in the cdf
    for (int i = 0; i < aPdfVec.size(); i++){
       running_sum += aPdfVec[i];
       aTargetVec.push_back(running_sum);
    }
    return running_sum;
}

template float MathUtil::genCDF<float>(const std::vector<float> &aPdfVec,
                                       std::vector<float> &aTargetVec);
template int MathUtil::genCDF<int>(const std::vector<int> &aPdfVec,
                                       std::vector<int> &aTargetVec);

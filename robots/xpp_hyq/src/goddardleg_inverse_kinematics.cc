/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <xpp_hyq/goddardleg_inverse_kinematics.h>

#include <cmath>
#include <map>

#include <xpp_states/cartesian_declarations.h>
#include <iostream>

namespace xpp {


GoddardlegInverseKinematics::Vector3d
GoddardlegInverseKinematics::GetJointAngles (const Vector3d& ee_pos_B, KneeBendSide bend, KneeBendSide side) const
{
  double aiis_angle, femur_angle, tibia_angle, rotation_x;

  Eigen::Vector3d xr;
  Eigen::Matrix3d R;

  // translate to the local coordinate of the attachment of the leg
  // and flip coordinate signs such that all computations can be done
  // for the front-left leg
  xr = ee_pos_B;


  // compute the HAA angle
  aiis_angle = atan2(xr[Y],-xr[Z]);
  rotation_x = -atan2(xr[Y],-xr[Z]);

  // rotate into the HFE coordinate system (rot around X)
  R << 1.0, 0.0, 0.0, 0.0, cos(rotation_x), -sin(rotation_x), 0.0, sin(rotation_x), cos(rotation_x);

  xr = (R * xr).eval();

  // translate into the HFE coordinate system (along Z axis)
  xr -= hfe_to_haa_z;  //distance of HFE to HAA in z direction

  // compute square of length from HFE to foot
  double tmp1 = pow(xr[X],2)+pow(xr[Z],2);


  // compute temporary angles (with reachability check)
  double lu = length_thigh;  // length of upper leg
  double ll = length_shank;  // length of lower leg
  double alpha = atan2(xr[Z],-xr[X]);

  double some_random_value_for_beta = (pow(lu,2)+tmp1-pow(ll,2))/(2.*lu*sqrt(tmp1)); // this must be between -1 and 1

  if (some_random_value_for_beta > 1) {
    some_random_value_for_beta = 1;
  }
  if (some_random_value_for_beta < -1) {
    some_random_value_for_beta = -1;
  }
  double beta = acos(some_random_value_for_beta);
  // compute Hip FE angle
  femur_angle = alpha + beta + (12.48*M_PI/180.0);


  double some_random_value_for_gamma = (pow(ll,2)+pow(lu,2)-tmp1)/(2.*ll*lu);
  // law of cosines give the knee angle
  if (some_random_value_for_gamma > 1) {
    some_random_value_for_gamma = 1;
  }
  if (some_random_value_for_gamma < -1) {
    some_random_value_for_gamma = -1;
  }
  double gamma  = acos(some_random_value_for_gamma);


  tibia_angle = -(100.58*M_PI/180.0) + gamma;
  
  // forward knee bend
  EnforceLimits(aiis_angle, HAA);
  EnforceLimits(femur_angle, HFE);
  EnforceLimits(tibia_angle, KFE);
  if (bend==Forward && side==Right)
    aiis_angle=-aiis_angle;
  if (bend==Backward && side==Right)
    aiis_angle=-aiis_angle;
    
  // else
    
  return Vector3d(aiis_angle, femur_angle, tibia_angle);
}

void
GoddardlegInverseKinematics::EnforceLimits (double& val, GoddardJointID joint) const
{
  // totally exaggerated joint angle limits
  const static double haa_min = -31.0;
  const static double haa_max =  31.0;

  const static double hfe_min = -62.85;
  const static double hfe_max =  0.0;

  const static double kfe_min = -81.98;
  const static double kfe_max =  0;

  // reduced joint angles for optimization
  static const std::map<GoddardJointID, double> max_range {
    {HAA, haa_max/180.0*M_PI},
    {HFE, hfe_max/180.0*M_PI},
    {KFE, kfe_max/180.0*M_PI}
  };

  // reduced joint angles for optimization
  static const std::map<GoddardJointID, double> min_range {
    {HAA, haa_min/180.0*M_PI},
    {HFE, hfe_min/180.0*M_PI},
    {KFE, kfe_min/180.0*M_PI}
  };

  double max = max_range.at(joint);
  val = val>max? max : val;

  double min = min_range.at(joint);
  val = val<min? min : val;
}

} /* namespace xpp */

/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <time.h>
#include <tf_conversions/tf_eigen.h>
#include <gtest/gtest.h>

using namespace tf;

double gen_rand(double min, double max)
{
  int rand_num = rand()%100+1;
  double result = min + (double)((max-min)*rand_num)/101.0;
  return result;
}

TEST(TFEigenConversions, tf_eigen_vector)
{
  tf::Vector3 t;
  t[0] = gen_rand(-10,10);
  t[1] = gen_rand(-10,10);
  t[2] = gen_rand(-10,10);

  Eigen::Vector3d k;
  vectorTFToEigen(t,k);

  ASSERT_NEAR(t[0],k[0],1e-6);
  ASSERT_NEAR(t[1],k[1],1e-6);
  ASSERT_NEAR(t[2],k[2],1e-6);
}

TEST(TFEigenConversions, tf_eigen_quaternion)
{
  tf::Quaternion t;
  t[0] = gen_rand(-1.0,1.0);
  t[1] = gen_rand(-1.0,1.0);
  t[2] = gen_rand(-1.0,1.0);
  t[3] = gen_rand(-1.0,1.0);
  t.normalize();
  Eigen::Quaterniond k;
  quaternionTFToEigen(t,k);

  ASSERT_NEAR(t[0],k.coeffs()(0),1e-6);
  ASSERT_NEAR(t[1],k.coeffs()(1),1e-6);
  ASSERT_NEAR(t[2],k.coeffs()(2),1e-6);
  ASSERT_NEAR(t[3],k.coeffs()(3),1e-6);
  ASSERT_NEAR(k.norm(),1.0,1e-10);
}

TEST(TFEigenConversions, tf_eigen_transform)
{
  tf::Transform t;
  tf::Quaternion tq;
  tq[0] = gen_rand(-1.0,1.0);
  tq[1] = gen_rand(-1.0,1.0);
  tq[2] = gen_rand(-1.0,1.0);
  tq[3] = gen_rand(-1.0,1.0);
  tq.normalize();
  t.setOrigin(tf::Vector3(gen_rand(-10,10),gen_rand(-10,10),gen_rand(-10,10)));
  t.setRotation(tq);

  Eigen::Affine3d k;
  transformTFToEigen(t,k);

  for(int i=0; i < 3; i++)
  {
    ASSERT_NEAR(t.getOrigin()[i],k.matrix()(i,3),1e-6);
    for(int j=0; j < 3; j++)
    {      
      ASSERT_NEAR(t.getBasis()[i][j],k.matrix()(i,j),1e-6);
    }
  }
  for (int col = 0 ; col < 3; col ++)
    ASSERT_NEAR(k.matrix()(3, col), 0, 1e-6);
  ASSERT_NEAR(k.matrix()(3,3), 1, 1e-6);
  
}

TEST(TFEigenConversions, eigen_tf_transform)
{
  tf::Transform t;
  Eigen::Affine3d k;
  Eigen::Quaterniond kq;
  kq.coeffs()(0) = gen_rand(-1.0,1.0);
  kq.coeffs()(1) = gen_rand(-1.0,1.0);
  kq.coeffs()(2) = gen_rand(-1.0,1.0);
  kq.coeffs()(3) = gen_rand(-1.0,1.0);
  kq.normalize();
  k.translate(Eigen::Vector3d(gen_rand(-10,10),gen_rand(-10,10),gen_rand(-10,10)));
  k.rotate(kq);

  transformEigenToTF(k,t);
  for(int i=0; i < 3; i++)
  {
    ASSERT_NEAR(t.getOrigin()[i],k.matrix()(i,3),1e-6);
    for(int j=0; j < 3; j++)
    {      
      ASSERT_NEAR(t.getBasis()[i][j],k.matrix()(i,j),1e-6);
    }
  }
}


int main(int argc, char **argv){
/* initialize random seed: */
  srand ( time(NULL) );
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

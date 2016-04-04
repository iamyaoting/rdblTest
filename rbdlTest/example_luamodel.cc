#include "RigidCharacter.h"


int main()
{
	rbdl_check_api_version(RBDL_API_VERSION);
	Model* model = NULL;

	RigdidCharacter character;
	character.readRbsFile("D:\\Projects\\C++\\Animation\\config\\bipV2Yaoting.rbs");

	system("pause");
	return 1;
}


///*
//* RBDL - Rigid Body Dynamics Library
//* Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
//*
//* Licensed under the zlib license. See LICENSE for more details.
//*/
//#include <iostream>
//#include <rbdl/rbdl.h>
//
//#include <cstdio>
//
//using namespace RigidBodyDynamics;
//using namespace RigidBodyDynamics::Math;
//
//int main(int argc, char* argv[]) {
//	rbdl_check_api_version(RBDL_API_VERSION);
//	Model* model = NULL;
//	unsigned int body_a_id, body_b_id;
//	Body body_a, body_b;
//	Joint joint_a, joint_b;
//	model = new Model();
//	model->gravity = Vector3d(0., -9.8, 0.);
//	body_a = Body(1.814, Vector3d(0.16, 0.0, 0.0), Vector3d(0.002173, 0.009104, 0.009104));
//	joint_a = Joint(
//		JointTypeRevolute,
//		Vector3d(0.0, 0.0, 1.0)
//		);
//
//	body_a_id = model->AddBody(0, Xtrans(Vector3d(0, 0., 0.)), joint_a, body_a);
//
//	body_b = Body(1.526, Vector3d(0.14, 0, 0), Vector3d(0.001640, 0.006753, 0.006753));
//	joint_b = Joint(
//		JointTypeRevolute,
//		Vector3d(0.0, 0.0, 1.0)
//		);
//		
//	body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(0.32, 0, 0)), joint_b, body_b);
//
//	VectorNd Q = VectorNd::Zero(model->dof_count);
//	VectorNd QDot = VectorNd::Zero(model->dof_count);
//	VectorNd Tau = VectorNd::Zero(model->dof_count);
//	VectorNd QDDot = VectorNd::Zero(model->dof_count);
//		
//	// read the data file
//	FILE *fp = NULL;
//	
//	//按读方式打开由argv[1]指出的文件
//	if ((fp = fopen("d:\\data.txt", "r")) == NULL)
//	{
//		printf("The file <%s> can not be opened.\n", argv[1]);//打开操作不成功
//	}
//			
//	VectorNd tmpQDDot = VectorNd::Zero(model->dof_count);
//	while (-1 != fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &Q[0], &QDot[0], &QDDot[0], &Tau[0], &Q[1], &QDot[1], &QDDot[1], &Tau[1])) //判断刚读取的字符是否是文件结束符
//	{
//	    //ForwardDynamicsLagrangian(*model, Q, QDot, Tau, tmpQDDot);
//		//std::cout << "RBDL(lagrangian)"<< tmpQDDot.transpose();
//		ForwardDynamics(*model, Q, QDot, Tau, tmpQDDot);
//		std::cout << "RBDL:" << tmpQDDot.transpose() << "  Matlab:" << QDDot.transpose() << std::endl;
//
//		//InverseDynamics(*model, Q, QDot, QDDot, Tau);
//		//std::cout << Tau.transpose() << std::endl;
//	} //完成将fp所指文件的内容输出到屏幕上显示
//	
//	fclose(fp); //关闭fp所指文件		
//	delete model;
//	system("pause");
//	return 0;
//}

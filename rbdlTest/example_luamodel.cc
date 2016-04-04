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
//	//������ʽ����argv[1]ָ�����ļ�
//	if ((fp = fopen("d:\\data.txt", "r")) == NULL)
//	{
//		printf("The file <%s> can not be opened.\n", argv[1]);//�򿪲������ɹ�
//	}
//			
//	VectorNd tmpQDDot = VectorNd::Zero(model->dof_count);
//	while (-1 != fscanf(fp, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf", &Q[0], &QDot[0], &QDDot[0], &Tau[0], &Q[1], &QDot[1], &QDDot[1], &Tau[1])) //�жϸն�ȡ���ַ��Ƿ����ļ�������
//	{
//	    //ForwardDynamicsLagrangian(*model, Q, QDot, Tau, tmpQDDot);
//		//std::cout << "RBDL(lagrangian)"<< tmpQDDot.transpose();
//		ForwardDynamics(*model, Q, QDot, Tau, tmpQDDot);
//		std::cout << "RBDL:" << tmpQDDot.transpose() << "  Matlab:" << QDDot.transpose() << std::endl;
//
//		//InverseDynamics(*model, Q, QDot, QDDot, Tau);
//		//std::cout << Tau.transpose() << std::endl;
//	} //��ɽ�fp��ָ�ļ��������������Ļ����ʾ
//	
//	fclose(fp); //�ر�fp��ָ�ļ�		
//	delete model;
//	system("pause");
//	return 0;
//}

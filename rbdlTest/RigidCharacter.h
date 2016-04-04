#pragma once
#include <iostream>
#include "RigidBody.h"
#include <cstdio>
#include <vector>

using namespace std;

class Joint
{
public:
	char name[200];
	char parent[200];
	char child[200];
	double cJPos[3];
	double pJPos[3];
	double transFormation[3];	// ����rbdl�йؽڵĲ�������ʾ�ؽ��ڸ���body��������ϵ�е�λ��
public:
	void loadFromFile(FILE* f);
};

class RigdidCharacter
{
public:
	vector<RigidBody*> rigidbodys;	
	vector<Joint*> joints;
	char rootName[200];
	Body *root = NULL;

	RigdidCharacter();
	~RigdidCharacter();
	void readRbsFile(char *fileName);

	void creatFixedModel();		// �������ؽڹ̶��ĹǼ�

private:
	void loadJointsFromFile(FILE* f);
	void fixRigidCharacterJoint();			// ��ҪΪ����Ӧrbdl��ʹ�ã����ú�rbdl����صĲο�ϵ�Ĳ���
	RigidBody* getRigdidBody(char* name);
};

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
	double transFormation[3];	// 用语rbdl中关节的参数，表示关节在父亲body所在坐标系中的位置
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

	void creatFixedModel();		// 建立根关节固定的骨架

private:
	void loadJointsFromFile(FILE* f);
	void fixRigidCharacterJoint();			// 主要为了适应rbdl库使用，设置和rbdl中相关的参考系的参数
	RigidBody* getRigdidBody(char* name);
};

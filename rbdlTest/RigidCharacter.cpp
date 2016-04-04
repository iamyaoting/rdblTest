#include "RigidCharacter.h"

RigdidCharacter::RigdidCharacter()
{
}


RigdidCharacter::~RigdidCharacter()
{
	for (int i = 0; i < rigidbodys.size(); ++i)
	{
		delete rigidbodys[i];
	}
	rigidbodys.clear();

	for (int i = 0; i < joints.size(); ++i)
	{
		delete joints[i];
	}
	joints.clear();

	if (root)
	{
		delete root;
	}
}

void RigdidCharacter::readRbsFile(char * fileName)
{
	FILE *f = fopen(fileName, "r");
	
	char buffer[200];
	RigidBody* newBody = NULL;
	//ArticulatedFigure* newFigure = NULL;


	while (!feof(f)) {
		//get a line from the file...
		fgets(buffer, 200, f);

		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
		case RB_RB:
			////create a new rigid body and have it load its own info...
			//newBody = new RigidBody();
			//newBody->loadFromFile(f);
			//objects.push_back(newBody);
			break;
		case RB_ARB:
			//create a new articulated rigid body and have it load its own info...
			newBody = new RigidBody();
			newBody->loadFromFile(f);
			//remember it as an articulated rigid body to be able to link it with other ABs later on
			rigidbodys.push_back(newBody);
			break;
		case RB_ARTICULATED_FIGURE:
			loadJointsFromFile(f);
			////we have an articulated figure to worry about...
			//newFigure = new ArticulatedFigure();
			//AFs.push_back(newFigure);
			//newFigure->loadFromFile(f, this);
			//newFigure->addJointsToList(&jts);
			break;
		case RB_NOT_IMPORTANT:
			if (strlen(line) != 0 && line[0] != '#')
				printf("Ignoring input line: \'%s\'\n", line);
			break;
		default:
			printf("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	
	fclose(f);
}

void RigdidCharacter::creatFixedModel()
{
	
}

void RigdidCharacter::loadJointsFromFile(FILE * f)
{
	if (f == NULL)
		printf("Invalid file pointer.");
	//if (world == NULL)
	//	printf("A valid physical world must be passed in as a parameter");
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];
	Joint* tempJoint;

	//this is where it happens.
	while (!feof(f)) {
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			printf("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
		case RB_ROOT:
			sscanf(line, "%s", rootName);
			//if (root != NULL)
			//	printf("This articulated figure already has a root");
			//root = world->getARBByName(tempName);
			//if (root == NULL)
			//	printf("The articulated rigid body \'%s\' cannot be found!", tempName);
			break;
		case RB_JOINT_TYPE_UNIVERSAL:
			tempJoint = new Joint();
			tempJoint->loadFromFile(f);
			joints.push_back(tempJoint);
			/*tempJoint = new UniversalJoint(line);
			tempJoint->loadFromFile(f, world);
			tempJoint->child->AFParent = this;
			tempJoint->parent->AFParent = this;*/
			break;
		case RB_JOINT_TYPE_HINGE:
			tempJoint = new Joint();
			tempJoint->loadFromFile(f);
			joints.push_back(tempJoint);
			/*tempJoint = new HingeJoint(line);
			tempJoint->loadFromFile(f, world);
			tempJoint->child->AFParent = this;
			tempJoint->parent->AFParent = this;*/
			break;
		case RB_JOINT_TYPE_BALL_IN_SOCKET:
			tempJoint = new Joint();
			tempJoint->loadFromFile(f);
			joints.push_back(tempJoint);
			/*tempJoint = new BallInSocketJoint(line);
			tempJoint->loadFromFile(f, world);
			tempJoint->child->AFParent = this;
			tempJoint->parent->AFParent = this;*/
			break;
		case RB_END_ARTICULATED_FIGURE:
			//make sure that the root does not have a parent, otherwise we'll end up with loops in the articulated figure]
			/*if (root->pJoint != NULL)
				printf("The root of the articulated figure is not allowed to have a parent!");*/
			return;//and... done
			break;
		case RB_NOT_IMPORTANT:
			if (strlen(line) != 0 && line[0] != '#')
				printf("Ignoring input line: \'%s\'\n", line);
			break;
		default:
			printf("Incorrect articulated body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	printf("Incorrect articulated body input file! No /ArticulatedFigure found");
}

void RigdidCharacter::fixRigidCharacterJoint()
{
	// 设置body中com的位置
	// Root
	auto tmpRoot = getRigdidBody(rootName);
	tmpRoot->com[0] = 0;
	tmpRoot->com[1] = 0;
	tmpRoot->com[2] = 0;
	

	// 其他body
	for (int j = 0; j < joints.size(); j++)
	{
		auto tmpBody = getRigdidBody(joints[j]->child);
		tmpBody->com[0] = joints[j]->cJPos[0];
		tmpBody->com[1] = joints[j]->cJPos[1];
		tmpBody->com[2] = joints[j]->cJPos[2];
	}


	// 关节部分
	for (int j = 0; j < joints.size(); ++j)
	{
		auto tmpBody = getRigdidBody(joints[j]->parent);
		joints[j]->transFormation[0] = tmpBody->com[0] + joints[j]->pJPos[0];
		joints[j]->transFormation[1] = tmpBody->com[1] + joints[j]->pJPos[1];
		joints[j]->transFormation[2] = tmpBody->com[2] + joints[j]->pJPos[2];
	}

}

RigidBody * RigdidCharacter::getRigdidBody(char * name)
{
	for (int i = 0; i < rigidbodys.size(); ++i)
	{
		if (strcmp(rigidbodys[i]->name, name))
		{
			return rigidbodys[i];
		}
	}
	return nullptr;
}

void Joint::loadFromFile(FILE * f)
{
	if (f == NULL)
		printf("Invalid file pointer.");
	//if (world == NULL)
	//	printf("A valid physical world must be passed in as a parameter");
	//have a temporary buffer used to read the file line by line...
	char buffer[200];
	char tempName[100];

	//this is where it happens.
	while (!feof(f)) {
		//get a line from the file...
		fgets(buffer, 200, f);
		if (strlen(buffer)>195)
			printf("The input file contains a line that is longer than ~200 characters - not allowed");
		char *line = lTrim(buffer);
		int lineType = getRBLineType(line);
		switch (lineType) {
		case RB_NAME:
			sscanf(line, "%s", this->name);
			break;
		case RB_PARENT:
			sscanf(line, "%s", parent);
			//if (parent != NULL)
			//	printf("This joint already has a parent");
			//parent = world->getARBByName(tempName);
			//if (parent == NULL)
			//	printf("The articulated rigid body \'%s\' cannot be found!", tempName);
			break;
		case RB_CHILD:
			sscanf(line, "%s", child);
			//if (child != NULL)
			//	printf("This joint already has a parent");
			//child = world->getARBByName(tempName);
			//if (child == NULL)
			//	printf("The articulated rigid body \'%s\' cannot be found!", tempName);
			break;
		case RB_CPOS:
			sscanf(line, "%lf %lf %lf", &cJPos[0], &cJPos[1], &cJPos[2]);
			break;
		case RB_PPOS:
			sscanf(line, "%lf %lf %lf", &pJPos[0], &pJPos[1], &pJPos[2]);
			break;
		case RB_END_JOINT:
			//we now have to link together the child and parent bodies
			/*if (child == NULL)
				printf("A joint has been found that does not have a child rigid body");
			if (parent == NULL)
				printf("A parent has been found that does not have a child rigid body");
			if (child->pJoint != NULL)
				printf("The child body \'%s\' already has a parent.", child->name);
			parent->cJoints.push_back(this);
			child->pJoint = this;*/
			return;//and... done
			break;
		case RB_JOINT_LIMITS:
			//readJointLimits(line);
			break;
		case RB_NOT_IMPORTANT:
			//if (strlen(line) != 0 && line[0] != '#')
			//	tprintf("Ignoring input line: \'%s\'\n", line);
			break;
		default:
			printf("Incorrect articulated body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	printf("Incorrect articulated body input file! No /ArticulatedFigure found");
}

#include "RigidBody.h"

int getRBLineType(char* &buffer) {
	KeyWord keywords[] = {
		{ "RigidBody", RB_RB },
		{ "/End", RB_END_RB },
		{ "A_RigidBody", RB_ARB },
		{ "mesh", RB_MESH_NAME },
		{ "mass", RB_MASS },
		{ "moi", RB_MOI },
		{ "colour", RB_COLOUR },
		{ "root", RB_ROOT },
		{ "ArticulatedFigure", RB_ARTICULATED_FIGURE },
		{ "child", RB_CHILD },
		{ "parent", RB_PARENT },
		{ "/ArticulatedFigure", RB_END_ARTICULATED_FIGURE },
		{ "name", RB_NAME },
		{ "Joint", RB_JOINT },
		{ "/Joint", RB_END_JOINT },
		{ "jointPPos", RB_PPOS },
		{ "jointCPos", RB_CPOS },
		{ "CDP_Sphere", RB_SPHERE },
		{ "CDP_Capsule", RB_CAPSULE },
		{ "CDP_Plane", RB_PLANE },
		{ "static", RB_LOCKED },
		{ "position", RB_POSITION },
		{ "orientation", RB_ORIENTATION },
		{ "velocity", RB_VELOCITY },
		{ "angularVelocity", RB_ANGULAR_VELOCITY },
		{ "frictionCoefficient", RB_FRICTION_COEFF },
		{ "restitutionCoefficient", RB_RESTITUTION_COEFF },
		{ "minBoundingSphere", RB_MIN_BDG_SPHERE },
		{ "hingeJoint" ,RB_JOINT_TYPE_HINGE },
		{ "jointLimits", RB_JOINT_LIMITS },
		{ "universalJoint", RB_JOINT_TYPE_UNIVERSAL },
		{ "ballInSocketJoint", RB_JOINT_TYPE_BALL_IN_SOCKET },
		{ "CDP_Box", RB_BOX },
		{ "planar", RB_PLANAR },
		{ "ODEGroundParameters", RB_ODE_GROUND_COEFFS },
		{ "softBody", RB_SOFT_BODY }
	};

	//declare a list of keywords
	int keyWordCount = sizeof(keywords) / sizeof(keywords[0]);

	for (int i = 0; i<keyWordCount; i++) {
		if (strncmp(buffer, keywords[i].keyWord, strlen(keywords[i].keyWord)) == 0) {
			buffer += strlen(keywords[i].keyWord);
			return keywords[i].retVal;
		}
	}

	return RB_NOT_IMPORTANT;
}

/**
This method returns a pointer to the first non-white space character location in the provided buffer
*/
inline char* lTrim(char* buffer) {
	while (*buffer == ' ' || *buffer == '\t' || *buffer == '\n' || *buffer == '\r')
		buffer++;
	return buffer;
}

void RigidBody::loadFromFile(FILE * f)
{
	if (f == NULL)
		printf("Invalid file pointer.");

	//have a temporary buffer used to read the file line by line...
	char buffer[200];
//	char meshName[200];

	//temporary variables that we may end up populating
	//double r, g, b, a;
	//Point3d p1, p2;
	//Vector3d n;
	double t1, t2, t3;
	double t;
//	GLMesh* tmpMesh;

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
			sscanf(line, "%s", name);
			break;
		case RB_MESH_NAME:
			//sscanf(line, "%s", meshName);
			//tmpMesh = OBJReader::loadOBJFile(meshName);
			//tmpMesh->computeNormals();
			//tmpMesh->dontUseTextureMapping();
			//meshes.push_back(tmpMesh);
			break;
		case RB_MASS:
			if (sscanf(line, "%lf", &t) != 1)
				printf("Incorrect rigid body input file - a mass needs to be specified if the 'mass' keyword is used.");
			mass = t;
			break;
		case RB_MOI:
			if (sscanf(line, "%lf %lf %lf", &t1, &t2, &t3) != 3)
				printf("Incorrect rigid body input file - the three principal moments of inertia need to be specified if the 'moi' keyword is used.");
			if (t1 <= 0 || t2 <= 0 || t3 <= 0)
				printf("Incorrect values for the principal moments of inertia.");
			moi[0] = t1; 
			moi[1] = t2;
			moi[2] = t3;
			break;
		case RB_END_RB:
			return;//and... done
			break;
		case RB_COLOUR:
			//if (sscanf(line, "%lf %lf %lf %lf", &r, &g, &b, &a) != 4)
			//	printf("Incorrect rigid body input file - colour parameter expects 4 arguments (colour %s)\n", line);
			//if (meshes.size()>0)
			//	meshes[meshes.size() - 1]->setColour(r, g, b, a);
			break;
		case RB_SPHERE:
			//if (sscanf(line, "%lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &r) != 4)
			//	printf("Incorrect rigid body input file - 4 arguments are required to specify a sphere collision detection primitive\n", line);
			//cdps.push_back(new SphereCDP(this, p1, r));
			break;
		case RB_CAPSULE:
			/*if (sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z, &r) != 7)
				printf("Incorrect rigid body input file - 7 arguments are required to specify a capsule collision detection primitive\n", line);
			cdps.push_back(new CapsuleCDP(this, p1, p2, r));*/
			break;
		case RB_BOX:
			/*if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &p1.x, &p1.y, &p1.z, &p2.x, &p2.y, &p2.z) != 6)
				printf("Incorrect rigid body input file - 6 arguments are required to specify a box collision detection primitive\n", line);
			cdps.push_back(new BoxCDP(this, p1, p2));*/
			break;
		case RB_PLANE:
			/*if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &n.x, &n.y, &n.z, &p1.x, &p1.y, &p1.z) != 6)
				printf("Incorrect rigid body input file - 6 arguments are required to specify a plane collision detection primitive\n", line);
			cdps.push_back(new PlaneCDP(this, n, p1));*/
			break;
		case RB_NOT_IMPORTANT:
			if (strlen(line) != 0 && line[0] != '#')
				printf("Ignoring input line: \'%s\'\n", line);
			break;
		case RB_LOCKED:
			//this->props.lockBody();
			break;
		case RB_POSITION:
			if (sscanf(line, "%lf %lf %lf", &position[0], &position[1], &position[2]) != 3)
				printf("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates position of a rigid body\n", line);
			break;
		case RB_ORIENTATION:
			/*if (sscanf(line, "%lf %lf %lf %lf", &t, &t1, &t2, &t3) != 4)
				printf("Incorrect rigid body input file - 4 arguments are required to specify the world coordinates orientation of a rigid body\n", line);
			state.orientation = Quaternion::getRotationQuaternion(t, Vector3d(t1, t2, t3).toUnit()) * state.orientation;*/
			break;
		case RB_VELOCITY:
			/*if (sscanf(line, "%lf %lf %lf", &state.velocity.x, &state.velocity.y, &state.velocity.z) != 3)
				printf("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates velocity of a rigid body\n", line);*/
			break;
		case RB_ANGULAR_VELOCITY:
			/*if (sscanf(line, "%lf %lf %lf", &state.angularVelocity.x, &state.angularVelocity.y, &state.angularVelocity.z) != 3)
				printf("Incorrect rigid body input file - 3 arguments are required to specify the world coordinates angular velocity of a rigid body\n", line);*/
			break;
		case RB_FRICTION_COEFF:
			/*if (sscanf(line, "%lf", &props.mu) != 1)
				printf("Incorrect rigid body input file - Expecting a value for the friction coefficient");
			if (props.mu<0)
				printf("Incorrect rigid body input file - Friction coefficient should be >= 0");*/
			break;
		case RB_RESTITUTION_COEFF:
			/*if (sscanf(line, "%lf", &props.epsilon) != 1)
				printf("Incorrect rigid body input file - Expecting a value for the restitution coefficient");
			if (props.epsilon<0 || props.epsilon>1)
				printf("Incorrect rigid body input file - restitution coefficient should be between 0 and 1");*/
			break;
		case RB_ODE_GROUND_COEFFS:
			/*if (sscanf(line, "%lf %lf", &t1, &t2) != 2)
				printf("Two parameters need to be provided for the ODE ground parameter settings");
			props.groundSoftness = t1;
			props.groundPenalty = t2;*/
			break;
		case RB_PLANAR:
			//props.isPlanar = true;
			break;
		default:
			printf("Incorrect rigid body input file: \'%s\' - unexpected line.", buffer);
		}
	}
	printf("Incorrect articulated body input file! No /End found");
}


RigidBody::RigidBody()
{
}


RigidBody::~RigidBody()
{
}

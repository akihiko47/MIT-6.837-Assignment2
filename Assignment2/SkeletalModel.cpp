#include "SkeletalModel.h"
#include <fstream>

#include <FL/Fl.H>

using namespace std;

void SkeletalModel::load(const char *skeletonFile, const char *meshFile, const char *attachmentsFile)
{
	loadSkeleton(skeletonFile);

	m_mesh.load(meshFile);
	m_mesh.loadAttachments(attachmentsFile, m_joints.size());

	computeBindWorldToJointTransforms();
	updateCurrentJointToWorldTransforms();
}

void SkeletalModel::draw(Matrix4f cameraMatrix, bool skeletonVisible)
{
	// draw() gets called whenever a redraw is required
	// (after an update() occurs, when the camera moves, the window is resized, etc)

	m_matrixStack.clear();
	m_matrixStack.push(cameraMatrix);

	if( skeletonVisible )
	{
		drawJoints();

		drawSkeleton();
	}
	else
	{
		// Clear out any weird matrix we may have been using for drawing the bones and revert to the camera matrix.
		glLoadMatrixf(m_matrixStack.top());

		// Tell the mesh to draw itself.
		m_mesh.draw();
	}
}

void SkeletalModel::loadSkeleton( const char* filename )
{
	// Load the skeleton from file here.

	ifstream input;
	input.open(filename);

	if (!input.is_open()) {
		throw std::runtime_error("Could not open file");
	}

	string line;
	while (getline(input, line)) {
		stringstream ss(line);
		Joint* joint = new Joint;

		float tx, ty, tz;
		int parent;

		ss >> tx >> ty >> tz >> parent;

		Matrix4f transformMatrix = Matrix4f::identity();
		transformMatrix(0, 3) = tx;
		transformMatrix(1, 3) = ty;
		transformMatrix(2, 3) = tz;
		joint->transform = transformMatrix;

		m_joints.push_back(joint);

		if (parent == -1) {
			m_rootJoint = joint;
		} else {
			m_joints[parent]->children.push_back(joint);
		}
	}

	input.close();
}

void SkeletalModel::drawJoints( )
{
	// Draw a sphere at each joint. You will need to add a recursive helper function to traverse the joint hierarchy.
	//
	// We recommend using glutSolidSphere( 0.025f, 12, 12 )
	// to draw a sphere of reasonable size.
	//
	// You are *not* permitted to use the OpenGL matrix stack commands
	// (glPushMatrix, glPopMatrix, glMultMatrix).
	// You should use your MatrixStack class
	// and use glLoadMatrixf() before your drawing call.

	drawJointWithChildren(m_rootJoint);
}

void SkeletalModel::drawJointWithChildren(Joint* joint) {

	m_matrixStack.push(joint->transform);
	glLoadMatrixf(m_matrixStack.top());
	glutSolidSphere(0.025f, 12, 12);

	if (joint->children.size() == 0) {
		return;
	} else {
		for (int i = 0; i < joint->children.size(); i++) {
			drawJointWithChildren(joint->children[i]);
			m_matrixStack.pop();
		}
	}
}

void SkeletalModel::drawSkeleton( )
{
	drawBoneOfJointWithChildren(m_rootJoint);
}

void SkeletalModel::drawBoneOfJointWithChildren(Joint* joint) {

	if (joint->children.size() == 0) {
		return;
	} else {
		for (int i = 0; i < joint->children.size(); i++) {
			// initializing bone transformation matrix
			Matrix4f boneTransform = Matrix4f::identity();

			// new basis calculation
			Vector3f rnd(0.0f, 0.0f, 1.0f);
			Vector3f z = joint->children[i]->transform.getCol(3).xyz().normalized();
			Vector3f y = Vector3f::cross(z, rnd).normalized();
			Vector3f x = Vector3f::cross(y, z).normalized();

			// matrix translating coordinates to basis of bone
			Matrix4f M = Matrix4f(
				x[0], y[0], z[0], 0.0f,
				x[1], y[1], z[1], 0.0f,
				x[2], y[2], z[2], 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f
			);

			// length of bone
			float l = joint->children[i]->transform.getCol(3).xyz().abs();

			// translate from origin
			boneTransform = boneTransform * Matrix4f::translation(-m_rootJoint->transform.getCol(3).xyz());

			// change basis
			boneTransform = boneTransform * M;

			// scale
			boneTransform = boneTransform * Matrix4f::scaling(0.025f, 0.025f, l);

			// translate froward to next joint
			boneTransform = boneTransform * Matrix4f::translation(0.0f, 0.0f, 0.5f);

			m_matrixStack.push(joint->transform);
			m_matrixStack.push(boneTransform);
			glLoadMatrixf(m_matrixStack.top());
			glutSolidCube(1.0f);
			m_matrixStack.pop();

			drawBoneOfJointWithChildren(joint->children[i]);
			m_matrixStack.pop();
		}
	}
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	// Set the rotation part of the joint's transformation matrix based on the passed in Euler angles.
}


void SkeletalModel::computeBindWorldToJointTransforms()
{
	// 2.3.1. Implement this method to compute a per-joint transform from
	// world-space to joint space in the BIND POSE.
	//
	// Note that this needs to be computed only once since there is only
	// a single bind pose.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	// 2.3.2. Implement this method to compute a per-joint transform from
	// joint space to world space in the CURRENT POSE.
	//
	// The current pose is defined by the rotations you've applied to the
	// joints and hence needs to be *updated* every time the joint angles change.
	//
	// This method should update each joint's bindWorldToJointTransform.
	// You will need to add a recursive helper function to traverse the joint hierarchy.
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.
}


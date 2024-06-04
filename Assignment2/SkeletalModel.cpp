#include "SkeletalModel.h"
#include <fstream>

#define PI 3.14159265359

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
		for (unsigned i = 0; i < joint->children.size(); i++) {
			drawJointWithChildren(joint->children[i]);
			m_matrixStack.pop();
		}
	}
}

void SkeletalModel::drawSkeleton( )
{	
	m_matrixStack.push(Matrix4f::translation(-m_rootJoint->transform.getCol(3).xyz()));

	drawBoneOfJointWithChildren(m_rootJoint);

	m_matrixStack.pop();
}

void SkeletalModel::drawBoneOfJointWithChildren(Joint* joint) {
	if (joint->children.size() != 0) {
		m_matrixStack.push(joint->transform);
		for (unsigned i = 0; i < joint->children.size(); i++) {
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

			// change basis
			m_matrixStack.push(M);

			// scale bone
			m_matrixStack.push(Matrix4f::scaling(0.025f, 0.025f, l));

			// translate froward to next joint
			m_matrixStack.push(Matrix4f::translation(0.0f, 0.0f, 0.5f));

			glLoadMatrixf(m_matrixStack.top());
			glutSolidCube(1.0f);
			m_matrixStack.pop();
			m_matrixStack.pop();
			m_matrixStack.pop();

			drawBoneOfJointWithChildren(joint->children[i]);
		}
		m_matrixStack.pop();
	}
}

void SkeletalModel::setJointTransform(int jointIndex, float rX, float rY, float rZ)
{
	Matrix4f Rx = Matrix4f::rotateX(rX);
	Matrix4f Ry = Matrix4f::rotateY(rY);
	Matrix4f Rz = Matrix4f::rotateZ(rZ);

	Matrix4f Rot = Rz * Ry * Rx;

	m_joints[jointIndex]->transform.setSubmatrix3x3(0, 0, Rot.getSubmatrix3x3(0, 0));
}

void SkeletalModel::computeBWithChildren(Joint* parent) {
	m_matrixStack.push(parent->transform);

	parent->bindWorldToJointTransform = m_matrixStack.top().inverse();

	for (unsigned i = 0; i < parent->children.size(); i++) {
		computeBWithChildren(parent->children[i]);
	}

	m_matrixStack.pop();
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
	m_matrixStack.clear();
	computeBWithChildren(m_rootJoint);
}

void SkeletalModel::computeTWithChildren(Joint* parent) {
	m_matrixStack.push(parent->transform);

	parent->currentJointToWorldTransform = m_matrixStack.top();

	for (unsigned i = 0; i < parent->children.size(); i++) {
		computeTWithChildren(parent->children[i]);
	}

	m_matrixStack.pop();
}

void SkeletalModel::updateCurrentJointToWorldTransforms()
{
	m_matrixStack.clear();
	computeTWithChildren(m_rootJoint);
}

void SkeletalModel::updateMesh()
{
	// 2.3.2. This is the core of SSD.
	// Implement this method to update the vertices of the mesh
	// given the current state of the skeleton.
	// You will need both the bind pose world --> joint transforms.
	// and the current joint --> world transforms.

	for (int i = 0; i < m_mesh.currentVertices.size(); i++) {

		Vector4f p(m_mesh.bindVertices[i][0], m_mesh.bindVertices[i][1], m_mesh.bindVertices[i][2], 1.0f);

		Vector4f p_new;
		for (int j = 0; j < m_joints.size() - 1; j++) {
			if (m_mesh.attachments[i][j] > 0.0f) {
				p_new = p_new + (m_mesh.attachments[i][j] * (m_joints[j + 1]->currentJointToWorldTransform * m_joints[j + 1]->bindWorldToJointTransform * p));
			}
		}

		m_mesh.currentVertices[i] = p_new.xyz();
	}
}


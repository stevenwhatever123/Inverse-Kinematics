#include <stdio.h>
#include <GL/glut.h>
#include <string>
#include <Windows.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stack>
#include <deque>
#include <stdio.h>
#include "BVH.h"
#include <Eigen/Dense>


#define PI 3.14159265

static int win_width, win_height;

bool animating = true;

float animationTime = 0.0f;

int frameNumber = 0;

int old_t = 0;

int numberoftimes = 0;

BVH *bvh = NULL;

static int drag_mouse_r = 0;
static int drag_mouse_l = 0; 
static int drag_mouse_m = 0; 
static int last_mouse_x, last_mouse_y;

static float   camera_yaw = 0.0f;     
static float   camera_pitch = -20.0f;  
static float   camera_distance = 5.0f; 

int counter = 0;
float error = 1000000;

int joint = 0;

float targetX = 0;
float targetY = 0;
float targetZ = 0;

void render(void);

void reshape(int w, int h);

void drawText(const char* message);

void SpecialInput(int key, int mx, int my);

void keyboard(unsigned char key, int mx, int my);

void idle(void);

void init(void);

void IKSolver(float x, float y, float z);

void IKSolverTwoHands(float x1, float y1, float z1, float x2, float y2, float z2);

void IKSolverTwoHandsIterations(float x1, float y1, float z1, float x2, float y2, float z2);

Eigen::MatrixXd getPosition(int selectedJoint);

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL);

	glutInitWindowPosition(100, 100);
	glutInitWindowSize(640, 480);
	glutCreateWindow("Animation Assignment");

	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutSpecialFunc(SpecialInput);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);

	init();

	glutMainLoop();
	return 0;
}

void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0.0, 0.0, -20.0);
	glRotatef(20.0, 1.0, 0.0, 0.0);
	glTranslatef(0.0, -0.5, 0.0);

	/*glTranslatef(0.0, 0.0, -10.0);
	glRotatef(20.0, 1.0, 20.0, 0.0);
	glRotatef(0.0, 0.0, 1.0, 0.0);
	glTranslatef(0.0, -0.5, 0.0);*/
	
	float size = 15.0f;

	glBegin(GL_QUADS);
	glColor3f(0.8, 0.8, 0.8);

	glVertex3f(-size, 0, -size);
	glVertex3f(size, 0, -size);
	glVertex3f(size, 0, size);
	glVertex3f(-size, 0, size);
	glEnd();
	

	// Color for model
	glColor3f(1.0, 0.0, 0.0);
	if (bvh != NULL)
		bvh->RenderFigure(frameNumber, 0.2f);

	// Color for text
	glColor3f(1.0, 0.0, 0.0);

	char message[64];

	if (bvh != NULL)
		sprintf(message, "BVH loaded!");
	else
		sprintf(message, "Press B to load BVH file");
	drawText(message);

	//glFlush();
	glutSwapBuffers();
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (double)w/h, 1, 500);

	win_width = w;
	win_height = h;
}

void drawText(const char *message)
{
	if (message == NULL)
		return;

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D(0.0, win_width, win_height, 0);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glRasterPos2i(8, 20);
	for(int i = 0; i < message[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, message[i]);

	glRasterPos2i(450, 20);
	std::string mes = "Press a to save file";
	for (int i = 0; i < mes[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);

	glRasterPos2i(8, 40);
	if(bvh != NULL)
		mes = "Current Selected Joint: " + bvh->joints[joint]->name;
	else
		mes = "Current Selected Joint: NULL";
	for (int i = 0; i < mes[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);

	glRasterPos2i(8, 60);
	Eigen::MatrixXd currentJoint = Eigen::MatrixXd::Zero(3, 1);
	if (bvh != NULL)
	{
		currentJoint = getPosition(joint);
		mes = "Current Selected Joint Position: " + std::to_string(currentJoint(0, 0))
			+ " " + std::to_string(currentJoint(1, 0))
			+ " " + std::to_string(currentJoint(2, 0));
	}
	else
		mes = "Current Selected Joint Position: 0 0 0";
	for (int i = 0; i < mes[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);

	glRasterPos2i(8, 80);
	if (bvh != NULL)
	{
		mes = std::to_string(currentJoint(0, 0));
		for (int i = 0; i < mes[i] != '\0'; i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);
	}

	glRasterPos2i(8, 100);
	if (bvh != NULL)
	{
		mes = std::to_string(currentJoint(1, 0));
		for (int i = 0; i < mes[i] != '\0'; i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);
	}

	glRasterPos2i(8, 120);
	if (bvh != NULL)
	{
		mes = std::to_string(currentJoint(2, 0));
		for (int i = 0; i < mes[i] != '\0'; i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);
	}


	glRasterPos2i(450, 40);
	mes = "Target X: " + std::to_string(targetX);
	for (int i = 0; i < mes[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);

	glRasterPos2i(450, 60);
	mes = "Target Y: " + std::to_string(targetY);
	for (int i = 0; i < mes[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);

	glRasterPos2i(450, 80);
	mes = "Target Z: " + std::to_string(targetZ);
	for (int i = 0; i < mes[i] != '\0'; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, mes[i]);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void SpecialInput(int key, int mx, int my)
{
	if (key == GLUT_KEY_UP)
	{
		targetY += 0.1;
	}

	if (key == GLUT_KEY_DOWN)
	{
		targetY -= 0.1;
	}

	if (key == GLUT_KEY_LEFT)
	{
		targetX -= 0.1;
	}

	if (key == GLUT_KEY_RIGHT)
	{
		targetX += 0.1;
	}
}

void keyboard(unsigned char key, int mx, int my)
{
	if (key == 'b')
	{
		OPENFILENAME ofn;

		// an another memory buffer to contain the file name
		wchar_t szFile[256];

		ZeroMemory(&ofn, sizeof(ofn));
		ofn.lStructSize = sizeof(ofn);
		ofn.hwndOwner = NULL;
		ofn.lpstrFile = szFile;
		ofn.lpstrFile[0] = '\0';
		ofn.nMaxFile = sizeof(szFile);
		ofn.lpstrFilter = L"BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
		ofn.nFilterIndex = 1;
		ofn.lpstrFileTitle = NULL;
		ofn.nMaxFileTitle = 0;
		ofn.lpstrInitialDir = NULL;
		ofn.lpstrTitle = L"Select BVH file";
		ofn.lpstrDefExt = L"bvh";
		ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

		bool readSuccess = GetOpenFileName(&ofn);
		if (readSuccess)
		{
			if (bvh != NULL)
				delete bvh;

			char buffer[256];
			wcstombs(buffer, szFile, sizeof(buffer));
			bvh = new BVH(buffer);

			if (!bvh->IsLoadSuccess())
			{
				delete bvh;
				bvh = NULL;
			}

			animationTime = 0.0f;
			frameNumber = 0;
		}
	}

	if (key == 'a')
	{
		OPENFILENAME ofn;

		char szFileName[256] = "";

		ZeroMemory(&ofn, sizeof(ofn));

		ofn.lStructSize = sizeof(ofn);
		ofn.lpstrFilter = L"BVH Motion Data (*.bvh)\0*.bvh\0All (*.*)\0*.*\0";
		ofn.lpstrFile = (LPWSTR)szFileName;
		ofn.nMaxFile = MAX_PATH;
		ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
		ofn.lpstrDefExt = (LPCWSTR)L"bvh";

		GetSaveFileName(&ofn);
		wprintf(L"the path is : %s\n", ofn.lpstrFile);

		char buffer[256];
		wcstombs(buffer, ofn.lpstrFile, sizeof(buffer));

		std::ofstream outfile(buffer);

		outfile << std::setprecision(6) << std::fixed;
		bvh->printTree(outfile, bvh->joints[0], "", true);

		outfile << "MOTION" << "\n";
		outfile << "Frames: " << bvh->GetNumFrame() << "\n";
		outfile << "Frame Time: " << bvh->GetInterval() << "\n";
		
		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			for (int j = 0; j < bvh->GetNumChannel(); j++)
			{
				outfile << bvh->GetMotion(i, j) << " ";
			}
			outfile << "\n";
		}

		outfile.close();
	}

	if (key == 'i')
	{
		for (int i = 0; i < bvh->GetNumJoint(); i++)
		{
			std::cout << bvh->joints[i]->name << ": " << i << "\n";
		}
	}

	if (key == '.')
	{
		if (bvh != NULL)
		{
			if (joint < bvh->GetNumJoint() - 1)
				joint++;
			std::cout << joint << "\n";
		}
	}

	if (key == ',')
	{
		if (bvh != NULL)
		{
			if (joint > 0)
				joint--;
			std::cout << joint << "\n";
		}
	}

	if (key == '[')
	{
		targetZ -= 0.1;
	}

	if (key == ']')
	{
		targetZ += 0.1;
	}

	if (key == 'f')
	{
		IKSolver(targetX, targetY, targetZ);
	}

	if (key == 't')
	{
		IKSolverTwoHands(2, -1, -0.128156, -1, 0, -0.128156);
	}

	if (key == 'g')
	{
		//IKSolverTwoHandsIterations(-2, 1.5, -0.128156, 2, 1, -0.128156);
	}

	if (key == 'v')
	{
		for (int frameN = 0; frameN < bvh->GetNumFrame(); frameN++)
		{
			Eigen::MatrixXd allPositionVector = Eigen::MatrixXd::Zero(bvh->GetNumChannel(), 1);

			// Ignore first three channel because It's position channel
			Eigen::MatrixXd allRotationVector =
				Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 1);

			for (int i = 3; i < bvh->GetNumChannel() / 3; i++)
			{
				// Flip zyx order to xyz
				allRotationVector((i - 3) * 3, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3 + 2);
				allRotationVector((i - 3) * 3 + 1, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3 + 1);
				allRotationVector((i - 3) * 3 + 2, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3);
			}

			for (int i = 0; i < bvh->GetNumJoint(); i++)
			{
				BVH::Joint currentJoint = *bvh->joints[i];

				Eigen::MatrixXd currentPosition = Eigen::MatrixXd::Zero(4, 1);

				bool hasParent = true;


				currentPosition(0, 0) = bvh->GetMotion(frameN, 0) * 0.2f;
				currentPosition(1, 0) = bvh->GetMotion(frameN, 1) * 0.2f;
				currentPosition(2, 0) = bvh->GetMotion(frameN, 2) * 0.2f;
				currentPosition(3, 0) = 1;

				if (bvh->joints[i]->parent == NULL)
				{
					//currentJoint = *bvh->joints[i];
					hasParent = false;
				}
				else
				{
					//currentJoint = *bvh->joints[i]->parent;
					currentJoint = *bvh->joints[i];
				}

				std::vector<Eigen::Matrix4Xd> rotationMatrices;
				std::vector<Eigen::Matrix4Xd> translationMatrices;
				do
				{
					double zRo;
					double yRo;
					double xRo;

					xRo = allRotationVector(currentJoint.index * 3, 0) * PI / 180;
					yRo = allRotationVector(currentJoint.index * 3 + 1, 0) * PI / 180;
					zRo = allRotationVector(currentJoint.index * 3 + 2, 0) * PI / 180;

					Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
					Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
					Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

					rotationMatrixZ(0, 0) = cos(zRo);
					rotationMatrixZ(0, 1) = -sin(zRo);
					rotationMatrixZ(1, 0) = sin(zRo);
					rotationMatrixZ(1, 1) = cos(zRo);
					rotationMatrixZ(2, 2) = 1;
					rotationMatrixZ(3, 3) = 1;

					rotationMatrixY(0, 0) = cos(yRo);
					rotationMatrixY(0, 2) = sin(yRo);
					rotationMatrixY(1, 1) = 1;
					rotationMatrixY(2, 0) = -sin(yRo);
					rotationMatrixY(2, 2) = cos(yRo);
					rotationMatrixY(3, 3) = 1;

					rotationMatrixX(0, 0) = 1;
					rotationMatrixX(1, 1) = cos(xRo);
					rotationMatrixX(1, 2) = -sin(xRo);
					rotationMatrixX(2, 1) = sin(xRo);
					rotationMatrixX(2, 2) = cos(xRo);
					rotationMatrixX(3, 3) = 1;

					Eigen::Matrix4Xd rotationMatrix =
						(rotationMatrixZ * rotationMatrixY) * rotationMatrixX;

					rotationMatrices.push_back(rotationMatrix);

					double offsetX = currentJoint.offset[0] * 0.2f;
					double offsetY = currentJoint.offset[1] * 0.2f;
					double offsetZ = currentJoint.offset[2] * 0.2f;

					Eigen::Matrix4Xd translationMatrix = Eigen::Matrix4Xd::Zero(4, 4);
					translationMatrix(0, 0) = 1;
					translationMatrix(1, 1) = 1;
					translationMatrix(2, 2) = 1;
					translationMatrix(3, 3) = 1;

					translationMatrix(0, 3) = offsetX;
					translationMatrix(1, 3) = offsetY;
					translationMatrix(2, 3) = offsetZ;

					translationMatrices.push_back(translationMatrix);

					if (currentJoint.parent == NULL)
						break;
					//hasParent = false;
					else
						currentJoint = *currentJoint.parent;
				} while (hasParent);

				Eigen::MatrixXd endPosition = currentPosition;

				for (int j = 0; j < rotationMatrices.size(); j++)
				{
					endPosition = rotationMatrices[j] * endPosition;
					endPosition = translationMatrices[j] * endPosition;
				}

				allPositionVector(i * 3, 0) = endPosition(0, 0);
				allPositionVector(i * 3 + 1, 0) = endPosition(1, 0);
				allPositionVector(i * 3 + 2, 0) = endPosition(2, 0);
			}

			Eigen::MatrixXd handTargetPosition = Eigen::MatrixXd::Zero(3, 1);
			handTargetPosition(0, 0) = targetX;
			handTargetPosition(1, 0) = targetY;
			handTargetPosition(2, 0) = targetZ;

			Eigen::MatrixXd endHandPosition = Eigen::MatrixXd::Zero(3, 1);
			endHandPosition(0, 0) = allPositionVector(joint * 3, 0);
			endHandPosition(1, 0) = allPositionVector(joint * 3 + 1, 0);
			endHandPosition(2, 0) = allPositionVector(joint * 3 + 2, 0);
			
			double deltaTheta = 1;
			double error = 1000;
			int counter = 0;

			Eigen::MatrixXd allEndRotationVector = 
				Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 1);

			do
			{
				// Compute Jacobian
				Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(3, allRotationVector.rows());
				for (int col = 0; col < allRotationVector.rows(); col++)
				{
					Eigen::MatrixXd p1Vector = Eigen::MatrixXd::Zero(4, 1);
					p1Vector(0, 0) = allPositionVector(col - (col % 3), 0);
					p1Vector(1, 0) = allPositionVector(col - (col % 3) + 1, 0);
					p1Vector(2, 0) = allPositionVector(col - (col % 3) + 2, 0);
					p1Vector(3, 0) = 1;

					double theta_i = allRotationVector(col, 0)* PI / 180;
					theta_i += deltaTheta;

					// Rotation Matrix for finding p2 Vector
					Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
					Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
					Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

					rotationMatrixZ(0, 0) = cos(theta_i);
					rotationMatrixZ(0, 1) = -sin(theta_i);
					rotationMatrixZ(1, 0) = sin(theta_i);
					rotationMatrixZ(1, 1) = cos(theta_i);
					rotationMatrixZ(2, 2) = 1;
					rotationMatrixZ(3, 3) = 1;

					rotationMatrixY(0, 0) = cos(theta_i);
					rotationMatrixY(0, 2) = sin(theta_i);
					rotationMatrixY(1, 1) = 1;
					rotationMatrixY(2, 0) = -sin(theta_i);
					rotationMatrixY(2, 2) = cos(theta_i);
					rotationMatrixY(3, 3) = 1;

					rotationMatrixX(0, 0) = 1;
					rotationMatrixX(1, 1) = cos(theta_i);
					rotationMatrixX(1, 2) = -sin(theta_i);
					rotationMatrixX(2, 1) = sin(theta_i);
					rotationMatrixX(2, 2) = cos(theta_i);
					rotationMatrixX(3, 3) = 1;

					Eigen::Matrix4Xd rotationMatrix;

					if (col % 3 == 0)
						rotationMatrix = rotationMatrixX;

					if (col % 3 == 1)
						rotationMatrix = rotationMatrixY;

					if (col % 3 == 2)
						rotationMatrix = rotationMatrixZ;


					// Find p2 by forward kinematics using rotation matrix and translation matrix above
					Eigen::MatrixXd p2Vector = Eigen::MatrixXd::Zero(4, 1);
					p2Vector = rotationMatrix * p1Vector;

					Jacobian(0, col) = (p2Vector(0, 0) - p1Vector(0, 0)) / deltaTheta;
					Jacobian(1, col) = (p2Vector(1, 0) - p1Vector(1, 0)) / deltaTheta;
					Jacobian(2, col) = (p2Vector(2, 0) - p1Vector(2, 0)) / deltaTheta;
				}

				Eigen::MatrixXd positionDifferenceMatrix =
					Eigen::MatrixXd::Zero(3, 1);
				positionDifferenceMatrix = handTargetPosition - endHandPosition;

				Eigen::MatrixXd inversedJacobian =
					//(Jacobian.transpose() * Jacobian).inverse() * Jacobian.transpose();
					Jacobian.completeOrthogonalDecomposition().pseudoInverse();

				Eigen::MatrixXd deltaThetas = inversedJacobian * positionDifferenceMatrix;

				// Convert it back to decimals
				for (int i = 0; i < deltaThetas.rows(); i++)
				{
					deltaThetas(i, 0) = deltaThetas(i, 0) * 180 / PI;
				}

				allRotationVector += deltaThetas;

				// ===============================================================================
				// Update Position by forward kinematics
				// ===============================================================================
				for (int i = 0; i < bvh->GetNumJoint(); i++)
				{
					BVH::Joint currentJoint = *bvh->joints[i];

					Eigen::MatrixXd currentPosition = Eigen::MatrixXd::Zero(4, 1);

					bool hasParent = true;

					currentPosition(0, 0) = bvh->GetMotion(frameN, 0) * 0.2f;
					currentPosition(1, 0) = bvh->GetMotion(frameN, 1) * 0.2f;
					currentPosition(2, 0) = bvh->GetMotion(frameN, 2) * 0.2f;
					currentPosition(3, 0) = 1;

					if (bvh->joints[i]->parent == NULL)
					{
						//currentJoint = *bvh->joints[i];
						hasParent = false;
					}
					else
					{
						currentJoint = *bvh->joints[i];
					}

					std::vector<Eigen::Matrix4Xd> rotationMatrices;
					std::vector<Eigen::Matrix4Xd> translationMatrices;
					while (hasParent)
					{
						double xRo;
						double yRo;
						double zRo;

						xRo = allRotationVector(currentJoint.index * 3, 0) * PI / 180;
						yRo = allRotationVector(currentJoint.index * 3 + 1, 0) * PI / 180;
						zRo = allRotationVector(currentJoint.index * 3 + 2, 0) * PI / 180;

						Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
						Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
						Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

						rotationMatrixZ(0, 0) = cos(zRo);
						rotationMatrixZ(0, 1) = -sin(zRo);
						rotationMatrixZ(1, 0) = sin(zRo);
						rotationMatrixZ(1, 1) = cos(zRo);
						rotationMatrixZ(2, 2) = 1;
						rotationMatrixZ(3, 3) = 1;

						rotationMatrixY(0, 0) = cos(yRo);
						rotationMatrixY(0, 2) = sin(yRo);
						rotationMatrixY(1, 1) = 1;
						rotationMatrixY(2, 0) = -sin(yRo);
						rotationMatrixY(2, 2) = cos(yRo);
						rotationMatrixY(3, 3) = 1;

						rotationMatrixX(0, 0) = 1;
						rotationMatrixX(1, 1) = cos(xRo);
						rotationMatrixX(1, 2) = -sin(xRo);
						rotationMatrixX(2, 1) = sin(xRo);
						rotationMatrixX(2, 2) = cos(xRo);
						rotationMatrixX(3, 3) = 1;

						Eigen::Matrix4Xd rotationMatrix =
							(rotationMatrixZ * rotationMatrixY) * rotationMatrixX;

						rotationMatrices.push_back(rotationMatrix);

						double offsetX = currentJoint.offset[0] * 0.2f;
						double offsetY = currentJoint.offset[1] * 0.2f;
						double offsetZ = currentJoint.offset[2] * 0.2f;

						Eigen::Matrix4Xd translationMatrix = Eigen::Matrix4Xd::Zero(4, 4);
						translationMatrix(0, 0) = 1;
						translationMatrix(1, 1) = 1;
						translationMatrix(2, 2) = 1;
						translationMatrix(3, 3) = 1;

						translationMatrix(0, 3) = offsetX;
						translationMatrix(1, 3) = offsetY;
						translationMatrix(2, 3) = offsetZ;

						translationMatrices.push_back(translationMatrix);

						if (currentJoint.parent == NULL)
							break;
						else
							currentJoint = *currentJoint.parent;
					}

					Eigen::MatrixXd endPosition = currentPosition;

					for (int j = 0; j < rotationMatrices.size(); j++)
					{
						endPosition = rotationMatrices[j] * endPosition;
						endPosition = translationMatrices[j] * endPosition;
					}

					allPositionVector(i * 3, 0) = endPosition(0, 0);
					allPositionVector(i * 3 + 1, 0) = endPosition(1, 0);
					allPositionVector(i * 3 + 2, 0) = endPosition(2, 0);
				}

				// Update current hand position
				endHandPosition(0, 0) = allPositionVector(27 * 3, 0);
				endHandPosition(1, 0) = allPositionVector(27 * 3 + 1, 0);
				endHandPosition(2, 0) = allPositionVector(27 * 3 + 2, 0);
				
				error = abs(endHandPosition(0, 0) - handTargetPosition(0, 0))
					  + abs(endHandPosition(1, 0) - handTargetPosition(1, 0))
					  + abs(endHandPosition(2, 0) - handTargetPosition(2, 0));
				counter++;

			} while (counter < 1000 && error > 0.03);

			std::cout << "Counter: " << counter << "\n";

			// Flip allEndRotationVector back to zyx order
			for (int i = 0; i < allRotationVector.rows() / 3 - 1; i++)
			{
				double temp = allRotationVector(i * 3);
				allRotationVector(i * 3) = allRotationVector(i * 3 + 2);
				allRotationVector(i * 3 + 2) = temp;
			}

			// Right hand
			BVH::Joint targetJoint = *bvh->joints[joint];
			// Loop through all parents
			BVH::Joint currentParentJoint = targetJoint;
			std::vector<int> parentStack;
			bool hasParentStack = true;
			while (hasParentStack)
			{
				if (currentParentJoint.parent == NULL)
				{
					parentStack.push_back(
						bvh->joints[currentParentJoint.index]->channels[3]->index);
					parentStack.push_back(
						bvh->joints[currentParentJoint.index]->channels[4]->index);
					parentStack.push_back(
						bvh->joints[currentParentJoint.index]->channels[5]->index);
					break;
				}
				else
				{
					parentStack.push_back(
						bvh->joints[currentParentJoint.index]->channels[0]->index);
					parentStack.push_back(
						bvh->joints[currentParentJoint.index]->channels[1]->index);
					parentStack.push_back(
						bvh->joints[currentParentJoint.index]->channels[2]->index);

					currentParentJoint = *currentParentJoint.parent;
				}
			}

			for (int j = 0; j < parentStack.size(); j++)
			{
				int channelIndex = parentStack[j];;
				bvh->SetMotion(frameN,
					parentStack[j],
					allRotationVector(channelIndex - 3, 0));
			}
		}
	}

	if (key == 'c')
	{
		int channel = 0;

		// Right shoulder
		for (int i = 0; i < bvh->joints.size(); i++) 
		{
			// Joint 24 is the right shoulder
			if (i == 25)
				break;
			channel += bvh->joints[i]->channels.size();
		}

		// Rotate 40 degrees on the right 
		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			bvh->SetMotion(i, channel, 30);
		}

		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			//bvh->SetMotion(i, channel - 3, 20);
		}

		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			//bvh->SetMotion(i, channel + 6, 30);
		}

		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			//bvh->SetMotion(i, channel + 9, 30);
		}

		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			//bvh->SetMotion(i, channel + 12, 30);
		}

		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			//bvh->SetMotion(i, channel + 15, 30);
		}

		/*
		channel = 0;
		// Left shoulder
		for (int i = 0; i < bvh->joints.size(); i++)
		{
			// Joint 17 is the left shoulder
			if (i == 17)
				break;
			channel += bvh->joints[i]->channels.size();
		}

		// Rotate 40 degrees on the right 
		for (int i = 0; i < bvh->GetNumFrame(); i++)
		{
			bvh->SetMotion(i, channel, 30);
		}
		*/
	}

	glutPostRedisplay();
}

void idle()
{
	// Calculate delta time
	int t;
	float deltaTime;
	t = glutGet(GLUT_ELAPSED_TIME);
	deltaTime = (t - old_t) / 1000.0;
	//std::cout << "delta time: " << deltaTime << "\n";
	old_t = t;

	if (animating)
	{
		animationTime += deltaTime;

		if (bvh != NULL)
		{
			frameNumber = animationTime / bvh->GetInterval();
			frameNumber = frameNumber % bvh->GetNumFrame();
		}
		else
			frameNumber = 0;

		glutPostRedisplay();
	}
}

void  init(void)
{
	old_t = glutGet(GLUT_ELAPSED_TIME);

	//glClearColor(0.5, 0.5, 0.8, 0.0);
	glClearColor(0.45, 0.84, 1.0, 0.0);
}

void IKSolver(float x, float y, float z)
{
	for (int frameN = 0; frameN < bvh->GetNumFrame(); frameN++)
	{
		Eigen::MatrixXd allPositionVector = Eigen::MatrixXd::Zero(bvh->GetNumJoint() * 3, 1);

		// Ignore first three channel because It's position channel
		Eigen::MatrixXd allRotationVector =
			Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 1);

		for (int i = 3; i < bvh->GetNumChannel() / 3; i++)
		{
			// Flip zyx order to xyz
			allRotationVector((i - 3) * 3, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3 + 2);
			allRotationVector((i - 3) * 3 + 1, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3 + 1);
			allRotationVector((i - 3) * 3 + 2, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3);
		}

		for (int i = 0; i < bvh->GetNumJoint(); i++)
		{
			BVH::Joint currentJoint = *bvh->joints[i];

			Eigen::MatrixXd currentPosition = Eigen::MatrixXd::Zero(4, 1);

			bool hasParent = true;


			currentPosition(0, 0) = bvh->GetMotion(frameN, 0) * 0.2f;
			currentPosition(1, 0) = bvh->GetMotion(frameN, 1) * 0.2f;
			currentPosition(2, 0) = bvh->GetMotion(frameN, 2) * 0.2f;
			currentPosition(3, 0) = 1;

			if (bvh->joints[i]->parent == NULL)
			{
				//currentJoint = *bvh->joints[i];
				hasParent = false;
			}
			else
			{
				//currentJoint = *bvh->joints[i]->parent;
				currentJoint = *bvh->joints[i];
			}

			std::vector<Eigen::Matrix4Xd> rotationMatrices;
			std::vector<Eigen::Matrix4Xd> translationMatrices;
			do
			{
				double zRo;
				double yRo;
				double xRo;

				xRo = allRotationVector(currentJoint.index * 3, 0) * PI / 180;
				yRo = allRotationVector(currentJoint.index * 3 + 1, 0) * PI / 180;
				zRo = allRotationVector(currentJoint.index * 3 + 2, 0) * PI / 180;

				Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
				Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
				Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

				rotationMatrixZ(0, 0) = cos(zRo);
				rotationMatrixZ(0, 1) = -sin(zRo);
				rotationMatrixZ(1, 0) = sin(zRo);
				rotationMatrixZ(1, 1) = cos(zRo);
				rotationMatrixZ(2, 2) = 1;
				rotationMatrixZ(3, 3) = 1;

				rotationMatrixY(0, 0) = cos(yRo);
				rotationMatrixY(0, 2) = sin(yRo);
				rotationMatrixY(1, 1) = 1;
				rotationMatrixY(2, 0) = -sin(yRo);
				rotationMatrixY(2, 2) = cos(yRo);
				rotationMatrixY(3, 3) = 1;

				rotationMatrixX(0, 0) = 1;
				rotationMatrixX(1, 1) = cos(xRo);
				rotationMatrixX(1, 2) = -sin(xRo);
				rotationMatrixX(2, 1) = sin(xRo);
				rotationMatrixX(2, 2) = cos(xRo);
				rotationMatrixX(3, 3) = 1;

				Eigen::Matrix4Xd rotationMatrix =
					(rotationMatrixZ * rotationMatrixY) * rotationMatrixX;

				rotationMatrices.push_back(rotationMatrix);

				double offsetX = currentJoint.offset[0] * 0.2f;
				double offsetY = currentJoint.offset[1] * 0.2f;
				double offsetZ = currentJoint.offset[2] * 0.2f;

				Eigen::Matrix4Xd translationMatrix = Eigen::Matrix4Xd::Zero(4, 4);
				translationMatrix(0, 0) = 1;
				translationMatrix(1, 1) = 1;
				translationMatrix(2, 2) = 1;
				translationMatrix(3, 3) = 1;

				translationMatrix(0, 3) = offsetX;
				translationMatrix(1, 3) = offsetY;
				translationMatrix(2, 3) = offsetZ;

				translationMatrices.push_back(translationMatrix);

				if (currentJoint.parent == NULL)
					break;
				//hasParent = false;
				else
					currentJoint = *currentJoint.parent;
			} while (hasParent);

			Eigen::MatrixXd endPosition = currentPosition;

			for (int j = 0; j < rotationMatrices.size(); j++)
			{
				endPosition = rotationMatrices[j] * endPosition;
				endPosition = translationMatrices[j] * endPosition;
			}

			allPositionVector(i * 3, 0) = endPosition(0, 0);
			allPositionVector(i * 3 + 1, 0) = endPosition(1, 0);
			allPositionVector(i * 3 + 2, 0) = endPosition(2, 0);
		}

		Eigen::MatrixXd handTargetPosition = Eigen::MatrixXd::Zero(3, 1);
		handTargetPosition(0, 0) = x;
		handTargetPosition(1, 0) = y;
		handTargetPosition(2, 0) = z;

		Eigen::MatrixXd endHandPosition = Eigen::MatrixXd::Zero(3, 1);
		endHandPosition(0, 0) = allPositionVector(joint * 3, 0);
		endHandPosition(1, 0) = allPositionVector(joint * 3 + 1, 0);
		endHandPosition(2, 0) = allPositionVector(joint * 3 + 2, 0);

		Eigen::MatrixXd allEndRotationVector =
			Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 0);

		// Compute Jacobian
		Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(3, allRotationVector.rows());
		for (int col = 0; col < allRotationVector.rows(); col++)
		{
			Eigen::MatrixXd p1Vector = Eigen::MatrixXd::Zero(4, 1);
			p1Vector(0, 0) = allPositionVector(col - (col % 3), 0);
			p1Vector(1, 0) = allPositionVector(col - (col % 3) + 1, 0);
			p1Vector(2, 0) = allPositionVector(col - (col % 3) + 2, 0);
			p1Vector(3, 0) = 1;

			double deltaTheta = 1;
			double theta_i = allRotationVector(col, 0) * PI / 180;
			theta_i += deltaTheta;

			// Rotation Matrix for finding p2 Vector
			Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
			Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
			Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

			rotationMatrixZ(0, 0) = cos(theta_i);
			rotationMatrixZ(0, 1) = -sin(theta_i);
			rotationMatrixZ(1, 0) = sin(theta_i);
			rotationMatrixZ(1, 1) = cos(theta_i);
			rotationMatrixZ(2, 2) = 1;
			rotationMatrixZ(3, 3) = 1;

			rotationMatrixY(0, 0) = cos(theta_i);
			rotationMatrixY(0, 2) = sin(theta_i);
			rotationMatrixY(1, 1) = 1;
			rotationMatrixY(2, 0) = -sin(theta_i);
			rotationMatrixY(2, 2) = cos(theta_i);
			rotationMatrixY(3, 3) = 1;

			rotationMatrixX(0, 0) = 1;
			rotationMatrixX(1, 1) = cos(theta_i);
			rotationMatrixX(1, 2) = -sin(theta_i);
			rotationMatrixX(2, 1) = sin(theta_i);
			rotationMatrixX(2, 2) = cos(theta_i);
			rotationMatrixX(3, 3) = 1;

			Eigen::Matrix4Xd rotationMatrix;
			//(rotationMatrixZ * rotationMatrixY * rotationMatrixX);

			if (col % 3 == 0)
				rotationMatrix = rotationMatrixX;

			if (col % 3 == 1)
				rotationMatrix = rotationMatrixY;

			if (col % 3 == 2)
				rotationMatrix = rotationMatrixZ;

			// Find p2 by forward kinematics using rotation matrix above
			Eigen::MatrixXd p2Vector = Eigen::MatrixXd::Zero(4, 1);
			p2Vector = rotationMatrix * p1Vector;

			Jacobian(0, col) = (p2Vector(0, 0) - p1Vector(0, 0)) / deltaTheta;
			Jacobian(1, col) = (p2Vector(1, 0) - p1Vector(1, 0)) / deltaTheta;
			Jacobian(2, col) = (p2Vector(2, 0) - p1Vector(2, 0)) / deltaTheta;
		}

		Eigen::MatrixXd positionDifferenceMatrix =
			Eigen::MatrixXd::Zero(3, 1);
		positionDifferenceMatrix = handTargetPosition - endHandPosition;

		Eigen::MatrixXd inversedJacobian =
			//(Jacobian.transpose() * Jacobian).inverse() * Jacobian.transpose();
			Jacobian.completeOrthogonalDecomposition().pseudoInverse();

		Eigen::MatrixXd deltaThetas = inversedJacobian * positionDifferenceMatrix;

		// Convert it back to decimals
		for (int i = 0; i < deltaThetas.rows(); i++)
		{
			deltaThetas(i, 0) = deltaThetas(i, 0) * 180 / PI;
		}

		allEndRotationVector
			//= (inversedJacobian * positionDifferenceMatrix) + allRotationVector;
			= deltaThetas + allRotationVector;

		// Flip allEndRotationVector back to zyx order
		for (int i = 0; i < allEndRotationVector.rows() / 3 - 1; i++)
		{
			double temp = allEndRotationVector(i * 3);
			allEndRotationVector(i * 3) = allEndRotationVector(i * 3 + 2);
			allEndRotationVector(i * 3 + 2) = temp;
		}

		// Right hand
		BVH::Joint targetJoint = *bvh->joints[joint];
		// Loop through all parents
		BVH::Joint currentParentJoint = targetJoint;
		std::vector<int> parentStack;
		bool hasParentStack = true;
		while (hasParentStack)
		{
			if (currentParentJoint.parent == NULL)
			{
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[3]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[4]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[5]->index);
				break;
			}
			else
			{
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[0]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[1]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[2]->index);
				currentParentJoint = *currentParentJoint.parent;
			}
		}

		for (int j = 0; j < parentStack.size(); j++)
		{
			int channelIndex = parentStack[j];
			bvh->SetMotion(frameN,
				channelIndex,
				allEndRotationVector(channelIndex - 3, 0));
		}

		error = abs(endHandPosition(0, 0) - handTargetPosition(0, 0))
				+ abs(endHandPosition(1, 0) - handTargetPosition(1, 0))
				+ abs(endHandPosition(2, 0) - handTargetPosition(2, 0));
		counter++;
	}
}

void IKSolverTwoHands(float x1, float y1, float z1, float x2, float y2, float z2)
{
	for (int frameN = 0; frameN < bvh->GetNumFrame(); frameN++)
	{
		Eigen::MatrixXd allPositionVector = Eigen::MatrixXd::Zero(bvh->GetNumJoint() * 3, 1);

		// Ignore first three channel because It's position channel
		Eigen::MatrixXd allRotationVector =
			Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 1);

		for (int i = 3; i < bvh->GetNumChannel() / 3; i++)
		{
			// Flip zyx order to xyz
			allRotationVector((i - 3) * 3, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3 + 2);
			allRotationVector((i - 3) * 3 + 1, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3 + 1);
			allRotationVector((i - 3) * 3 + 2, 0) = bvh->GetMotion(frameN, (i - 3) * 3 + 3);
		}

		for (int i = 0; i < bvh->GetNumJoint(); i++)
		{
			BVH::Joint currentJoint = *bvh->joints[i];

			Eigen::MatrixXd currentPosition = Eigen::MatrixXd::Zero(4, 1);

			bool hasParent = true;


			currentPosition(0, 0) = bvh->GetMotion(frameN, 0) * 0.2f;
			currentPosition(1, 0) = bvh->GetMotion(frameN, 1) * 0.2f;
			currentPosition(2, 0) = bvh->GetMotion(frameN, 2) * 0.2f;
			currentPosition(3, 0) = 1;

			if (bvh->joints[i]->parent == NULL)
			{
				//currentJoint = *bvh->joints[i];
				hasParent = false;
			}
			else
			{
				//currentJoint = *bvh->joints[i]->parent;
				currentJoint = *bvh->joints[i];
			}

			std::vector<Eigen::Matrix4Xd> rotationMatrices;
			std::vector<Eigen::Matrix4Xd> translationMatrices;
			do
			{
				double zRo;
				double yRo;
				double xRo;

				xRo = allRotationVector(currentJoint.index * 3, 0) * PI / 180;
				yRo = allRotationVector(currentJoint.index * 3 + 1, 0) * PI / 180;
				zRo = allRotationVector(currentJoint.index * 3 + 2, 0) * PI / 180;

				Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
				Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
				Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

				rotationMatrixZ(0, 0) = cos(zRo);
				rotationMatrixZ(0, 1) = -sin(zRo);
				rotationMatrixZ(1, 0) = sin(zRo);
				rotationMatrixZ(1, 1) = cos(zRo);
				rotationMatrixZ(2, 2) = 1;
				rotationMatrixZ(3, 3) = 1;

				rotationMatrixY(0, 0) = cos(yRo);
				rotationMatrixY(0, 2) = sin(yRo);
				rotationMatrixY(1, 1) = 1;
				rotationMatrixY(2, 0) = -sin(yRo);
				rotationMatrixY(2, 2) = cos(yRo);
				rotationMatrixY(3, 3) = 1;

				rotationMatrixX(0, 0) = 1;
				rotationMatrixX(1, 1) = cos(xRo);
				rotationMatrixX(1, 2) = -sin(xRo);
				rotationMatrixX(2, 1) = sin(xRo);
				rotationMatrixX(2, 2) = cos(xRo);
				rotationMatrixX(3, 3) = 1;

				Eigen::Matrix4Xd rotationMatrix =
					(rotationMatrixZ * rotationMatrixY) * rotationMatrixX;

				rotationMatrices.push_back(rotationMatrix);

				double offsetX = currentJoint.offset[0] * 0.2f;
				double offsetY = currentJoint.offset[1] * 0.2f;
				double offsetZ = currentJoint.offset[2] * 0.2f;

				Eigen::Matrix4Xd translationMatrix = Eigen::Matrix4Xd::Zero(4, 4);
				translationMatrix(0, 0) = 1;
				translationMatrix(1, 1) = 1;
				translationMatrix(2, 2) = 1;
				translationMatrix(3, 3) = 1;

				translationMatrix(0, 3) = offsetX;
				translationMatrix(1, 3) = offsetY;
				translationMatrix(2, 3) = offsetZ;

				translationMatrices.push_back(translationMatrix);

				if (currentJoint.parent == NULL)
					break;
				//hasParent = false;
				else
					currentJoint = *currentJoint.parent;
			} while (hasParent);

			Eigen::MatrixXd endPosition = currentPosition;

			for (int j = 0; j < rotationMatrices.size(); j++)
			{
				endPosition = rotationMatrices[j] * endPosition;
				endPosition = translationMatrices[j] * endPosition;
			}

			allPositionVector(i * 3, 0) = endPosition(0, 0);
			allPositionVector(i * 3 + 1, 0) = endPosition(1, 0);
			allPositionVector(i * 3 + 2, 0) = endPosition(2, 0);
		}

		// both hands needs a 6x1 matrix
		Eigen::MatrixXd handTargetPosition = Eigen::MatrixXd::Zero(6, 1);
		handTargetPosition(0, 0) = x1;
		handTargetPosition(1, 0) = y1;
		handTargetPosition(2, 0) = z1;
		handTargetPosition(3, 0) = x2;
		handTargetPosition(4, 0) = y2;
		handTargetPosition(5, 0) = z2;

		Eigen::MatrixXd endHandPosition = Eigen::MatrixXd::Zero(6, 1);
		// Right hand
		endHandPosition(0, 0) = allPositionVector(27 * 3, 0);
		endHandPosition(1, 0) = allPositionVector(27 * 3 + 1, 0);
		endHandPosition(2, 0) = allPositionVector(27 * 3 + 2, 0);
		// Left hand
		endHandPosition(3, 0) = allPositionVector(20 * 3, 0);
		endHandPosition(4, 0) = allPositionVector(20 * 3 + 1, 0);
		endHandPosition(5, 0) = allPositionVector(20 * 3 + 2, 0);

		std::cout << endHandPosition << "\n\n";

		Eigen::MatrixXd allEndRotationVector =
			Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 0);

		Eigen::MatrixXd allTempPositionVector = allPositionVector;

		for (int i = 0; i < allEndRotationVector.size(); i++)
		{
			if (i == allEndRotationVector.size() - 2)
			{
				allTempPositionVector(i * 3, 0) = allPositionVector(0, 0);
				allTempPositionVector(i * 3 + 1, 0) = allPositionVector(1, 0);
				allTempPositionVector(i * 3 + 2, 0) = allPositionVector(2, 0);
			}
			else
			{
				allTempPositionVector(i * 3, 0) = allPositionVector(i * 4, 0);
				allTempPositionVector(i * 3 + 1, 0) = allPositionVector(i * 4 + 1, 0);
				allTempPositionVector(i * 3 + 2, 0) = allPositionVector(i * 4 + 2, 0);
			}
		}

		// Compute Jacobian
		Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(6, allRotationVector.rows());
		for (int col = 0; col < allRotationVector.rows(); col++)
		{
			Eigen::MatrixXd p1Vector = Eigen::MatrixXd::Zero(4, 1);
			p1Vector(0, 0) = allPositionVector(col - (col % 3), 0);
			p1Vector(1, 0) = allPositionVector(col - (col % 3) + 1, 0);
			p1Vector(2, 0) = allPositionVector(col - (col % 3) + 2, 0);
			p1Vector(3, 0) = 1;

			double deltaTheta = 1;
			double theta_i = allRotationVector(col, 0) * PI / 180;
			theta_i += deltaTheta;

			// Rotation Matrix for finding p2 Vector
			Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
			Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
			Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

			rotationMatrixZ(0, 0) = cos(theta_i);
			rotationMatrixZ(0, 1) = -sin(theta_i);
			rotationMatrixZ(1, 0) = sin(theta_i);
			rotationMatrixZ(1, 1) = cos(theta_i);
			rotationMatrixZ(2, 2) = 1;
			rotationMatrixZ(3, 3) = 1;

			rotationMatrixY(0, 0) = cos(theta_i);
			rotationMatrixY(0, 2) = sin(theta_i);
			rotationMatrixY(1, 1) = 1;
			rotationMatrixY(2, 0) = -sin(theta_i);
			rotationMatrixY(2, 2) = cos(theta_i);
			rotationMatrixY(3, 3) = 1;

			rotationMatrixX(0, 0) = 1;
			rotationMatrixX(1, 1) = cos(theta_i);
			rotationMatrixX(1, 2) = -sin(theta_i);
			rotationMatrixX(2, 1) = sin(theta_i);
			rotationMatrixX(2, 2) = cos(theta_i);
			rotationMatrixX(3, 3) = 1;

			Eigen::Matrix4Xd rotationMatrix;
			//(rotationMatrixZ * rotationMatrixY * rotationMatrixX);

			if (col % 3 == 0)
				rotationMatrix = rotationMatrixX;

			if (col % 3 == 1)
				rotationMatrix = rotationMatrixY;

			if (col % 3 == 2)
				rotationMatrix = rotationMatrixZ;

			// Find p2 by forward kinematics using rotation matrix above
			Eigen::MatrixXd p2Vector = Eigen::MatrixXd::Zero(4, 1);
			p2Vector = rotationMatrix * p1Vector;

			Jacobian(0, col) = (p2Vector(0, 0) - p1Vector(0, 0)) / deltaTheta;
			Jacobian(1, col) = (p2Vector(1, 0) - p1Vector(1, 0)) / deltaTheta;
			Jacobian(2, col) = (p2Vector(2, 0) - p1Vector(2, 0)) / deltaTheta;

			Eigen::MatrixXd p3Vector = Eigen::MatrixXd::Zero(4, 1);
			p3Vector(0, 0) = allTempPositionVector(col - (col % 3), 0);
			p3Vector(1, 0) = allTempPositionVector(col - (col % 3) + 1, 0);
			p3Vector(2, 0) = allTempPositionVector(col - (col % 3) + 2, 0);
			p3Vector(3, 0) = 1;

			Eigen::MatrixXd p4Vector = Eigen::MatrixXd::Zero(4, 1);
			p4Vector = rotationMatrix * p3Vector;

			Jacobian(3, col) = (p4Vector(0, 0) - p3Vector(0, 0)) / deltaTheta;
			Jacobian(4, col) = (p4Vector(1, 0) - p3Vector(1, 0)) / deltaTheta;
			Jacobian(5, col) = (p4Vector(2, 0) - p3Vector(2, 0)) / deltaTheta;

		}


		Eigen::MatrixXd positionDifferenceMatrix =
			Eigen::MatrixXd::Zero(6, 1);
		positionDifferenceMatrix = handTargetPosition - endHandPosition;

		Eigen::MatrixXd inversedJacobian =
			//(Jacobian.transpose() * Jacobian).inverse() * Jacobian.transpose();
			Jacobian.completeOrthogonalDecomposition().pseudoInverse();

		Eigen::MatrixXd deltaThetas = inversedJacobian * positionDifferenceMatrix;

		// Convert back to decimals
		for (int i = 0; i < deltaThetas.rows(); i++)
		{
			deltaThetas(i, 0) = deltaThetas(i, 0) * 180 / PI;
		}

		allEndRotationVector
			= deltaThetas + allRotationVector;

		// Flip allEndRotationVector back to zyx order
		for (int i = 0; i < allEndRotationVector.rows() / 3 - 1; i++)
		{
			double temp = allEndRotationVector(i * 3);
			allEndRotationVector(i * 3) = allEndRotationVector(i * 3 + 2);
			allEndRotationVector(i * 3 + 2) = temp;
		}

		// Right hand
		BVH::Joint targetJoint = *bvh->joints[27];
		// Loop through all parents
		BVH::Joint currentParentJoint = targetJoint;
		std::vector<int> parentStack;
		bool hasParentStack = true;
		while (hasParentStack)
		{
			if (currentParentJoint.parent == NULL)
			{
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[3]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[4]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[5]->index);
				break;
			}
			else
			{
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[0]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[1]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[2]->index);
				currentParentJoint = *currentParentJoint.parent;
			}
		}

		for (int j = 0; j < parentStack.size(); j++)
		{
			int channelIndex = parentStack[j];
			bvh->SetMotion(frameN,
				channelIndex,
				allEndRotationVector(channelIndex - 3, 0));
		}

		// Left hand
		targetJoint = *bvh->joints[20];
		// Loop through all parents
		currentParentJoint = targetJoint;
		parentStack.clear();
		hasParentStack = true;
		while (hasParentStack)
		{
			if (currentParentJoint.parent == NULL)
			{
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[3]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[4]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[5]->index);
				break;
			}
			else
			{
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[0]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[1]->index);
				parentStack.push_back(
					bvh->joints[currentParentJoint.index]->channels[2]->index);
				currentParentJoint = *currentParentJoint.parent;
			}
		}

		for (int j = 0; j < parentStack.size(); j++)
		{
			int channelIndex = parentStack[j];
			bvh->SetMotion(frameN,
				channelIndex,
				allEndRotationVector(channelIndex - 3, 0));
		}
	}
}

Eigen::MatrixXd getPosition(int selectedJoint)
{
	Eigen::MatrixXd returnVal = Eigen::MatrixXd::Zero(3, 1);

	Eigen::MatrixXd allPositionVector = Eigen::MatrixXd::Zero(bvh->GetNumJoint() * 3, 1);

	// Ignore first three channel because It's position channel
	Eigen::MatrixXd allRotationVector =
		Eigen::MatrixXd::Zero(bvh->GetNumChannel() - 3, 1);

	for (int i = 3; i < bvh->GetNumChannel() / 3; i++)
	{
		// Flip zyx order to xyz
		allRotationVector((i - 3) * 3, 0) = bvh->GetMotion(frameNumber, (i - 3) * 3 + 3 + 2);
		allRotationVector((i - 3) * 3 + 1, 0) = bvh->GetMotion(frameNumber, (i - 3) * 3 + 3 + 1);
		allRotationVector((i - 3) * 3 + 2, 0) = bvh->GetMotion(frameNumber, (i - 3) * 3 + 3);
	}

	for (int i = 0; i < bvh->GetNumJoint(); i++)
	{
		BVH::Joint currentJoint = *bvh->joints[i];

		Eigen::MatrixXd currentPosition = Eigen::MatrixXd::Zero(4, 1);

		bool hasParent = true;


		currentPosition(0, 0) = bvh->GetMotion(frameNumber, 0) * 0.2f;
		currentPosition(1, 0) = bvh->GetMotion(frameNumber, 1) * 0.2f;
		currentPosition(2, 0) = bvh->GetMotion(frameNumber, 2) * 0.2f;
		currentPosition(3, 0) = 1;

		if (bvh->joints[i]->parent == NULL)
		{
			//currentJoint = *bvh->joints[i];
			hasParent = false;
		}
		else
		{
			//currentJoint = *bvh->joints[i]->parent;
			currentJoint = *bvh->joints[i];
		}

		std::vector<Eigen::Matrix4Xd> rotationMatrices;
		std::vector<Eigen::Matrix4Xd> translationMatrices;
		do
		{
			double zRo;
			double yRo;
			double xRo;

			xRo = allRotationVector(currentJoint.index * 3, 0) * PI / 180;
			yRo = allRotationVector(currentJoint.index * 3 + 1, 0) * PI / 180;
			zRo = allRotationVector(currentJoint.index * 3 + 2, 0) * PI / 180;

			Eigen::Matrix4Xd rotationMatrixZ = Eigen::Matrix4Xd::Zero(4, 4);
			Eigen::Matrix4Xd rotationMatrixY = Eigen::Matrix4Xd::Zero(4, 4);
			Eigen::Matrix4Xd rotationMatrixX = Eigen::Matrix4Xd::Zero(4, 4);

			rotationMatrixZ(0, 0) = cos(zRo);
			rotationMatrixZ(0, 1) = -sin(zRo);
			rotationMatrixZ(1, 0) = sin(zRo);
			rotationMatrixZ(1, 1) = cos(zRo);
			rotationMatrixZ(2, 2) = 1;
			rotationMatrixZ(3, 3) = 1;

			rotationMatrixY(0, 0) = cos(yRo);
			rotationMatrixY(0, 2) = sin(yRo);
			rotationMatrixY(1, 1) = 1;
			rotationMatrixY(2, 0) = -sin(yRo);
			rotationMatrixY(2, 2) = cos(yRo);
			rotationMatrixY(3, 3) = 1;

			rotationMatrixX(0, 0) = 1;
			rotationMatrixX(1, 1) = cos(xRo);
			rotationMatrixX(1, 2) = -sin(xRo);
			rotationMatrixX(2, 1) = sin(xRo);
			rotationMatrixX(2, 2) = cos(xRo);
			rotationMatrixX(3, 3) = 1;

			Eigen::Matrix4Xd rotationMatrix =
				(rotationMatrixZ * rotationMatrixY) * rotationMatrixX;

			rotationMatrices.push_back(rotationMatrix);

			double offsetX = currentJoint.offset[0] * 0.2f;
			double offsetY = currentJoint.offset[1] * 0.2f;
			double offsetZ = currentJoint.offset[2] * 0.2f;

			Eigen::Matrix4Xd translationMatrix = Eigen::Matrix4Xd::Zero(4, 4);
			translationMatrix(0, 0) = 1;
			translationMatrix(1, 1) = 1;
			translationMatrix(2, 2) = 1;
			translationMatrix(3, 3) = 1;

			translationMatrix(0, 3) = offsetX;
			translationMatrix(1, 3) = offsetY;
			translationMatrix(2, 3) = offsetZ;

			translationMatrices.push_back(translationMatrix);

			if (currentJoint.parent == NULL)
				break;
			//hasParent = false;
			else
				currentJoint = *currentJoint.parent;
		} while (hasParent);

		Eigen::MatrixXd endPosition = currentPosition;

		for (int j = 0; j < rotationMatrices.size(); j++)
		{
			endPosition = rotationMatrices[j] * endPosition;
			endPosition = translationMatrices[j] * endPosition;
		}

		allPositionVector(i * 3, 0) = endPosition(0, 0);
		allPositionVector(i * 3 + 1, 0) = endPosition(1, 0);
		allPositionVector(i * 3 + 2, 0) = endPosition(2, 0);
	}

	returnVal(0, 0) = allPositionVector(selectedJoint * 3, 0);
	returnVal(1, 0) = allPositionVector(selectedJoint * 3 + 1, 0);
	returnVal(2, 0) = allPositionVector(selectedJoint * 3 + 2, 0);

	return returnVal;
}

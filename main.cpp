#include <stdio.h>
#include <GL/glut.h>
#include <string>
#include <Windows.h>
#include <iostream>
#include "BVH.h"

static int win_width, win_height;

bool animating = true;

float animationTime = 0.0f;

int frameNumber = 0;

int old_t = 0;

BVH *bvh = NULL;

void render(void);

void reshape(int w, int h);

void drawText(const char* message);

void keyboard(unsigned char key, int mx, int my);

void idle(void);

void init(void);

int main(int argc, char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_STENCIL);

	glutInitWindowPosition(100, 100);
	glutInitWindowSize(640, 480);
	glutCreateWindow("Animation Assignment");

	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
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
	glRotatef(0.0, 0.0, 1.0, 0.0);
	glTranslatef(0.0, -0.5, 0.0);

	
	/*float  size = 1.5f;
	int  num_x = 15, num_z = 15;
	double  ox, oz;
	glBegin(GL_QUADS);
	glNormal3d(0.0, 1.0, 0.0);
	ox = -(num_x * size) / 2;
	for (int x = 0; x < num_x; x++, ox += size)
	{
		oz = -(num_z * size) / 2;
		for (int z = 0; z < num_z; z++, oz += size)
		{
			if (((x + z) % 2) == 0)
				glColor3f(1.0, 1.0, 1.0);
			else
				glColor3f(0.8, 0.8, 0.8);
			glVertex3d(ox, 0.0, oz);
			glVertex3d(ox, 0.0, oz + size);
			glVertex3d(ox + size, 0.0, oz + size);
			glVertex3d(ox + size, 0.0, oz);
		}
	}
	glEnd();*/
	
	
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
		sprintf(message, "BVH loaded fuck-face!");
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

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
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
		// This is the root joint
		std::cout << bvh->joints[0]->name << "\n";
		std::cout << "Offset: " << "\n";
		std::cout << bvh->joints[0]->offset[0] << "\n";
		std::cout << bvh->joints[0]->offset[1] << "\n";
		std::cout << bvh->joints[0]->offset[2] << "\n";
		std::cout << "\n";

		for (int i = 0; i < bvh->joints[0]->children.size(); i++)
		{
			BVH::Joint nextJoint = *bvh->joints[0]->children[i];
			bool loopChildren = true;
			while (loopChildren)
			{
				std::cout << nextJoint.name << "\n";
				std::cout << "Offset: " << "\n";
				std::cout << nextJoint.offset[0] << "\n";
				std::cout << nextJoint.offset[1] << "\n";
				std::cout << nextJoint.offset[2] << "\n";
				std::cout << "\n";

				if (nextJoint.children.size() < 1)
					loopChildren = false;
				else
					nextJoint = *nextJoint.children[0];
			}
		}
		
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

/* 
 * File:   GLSimpleObjects.h
 * Author: alonso
 *
 * Created on August 22, 2012, 11:29 AM
 */

#ifndef GLSIMPLEOBJECTS_H
#define	GLSIMPLEOBJECTS_H

////////////////////////////////////////////////////////////////////////////////
void drawLine(double x1, double y1, double x2, double y2)
{
	glBegin(GL_LINES);
		glVertex2f(x1,y1);
		glVertex2f(x2,y2);
	glEnd();
}

////////////////////////////////////////////////////////////////////////////////
void drawLine3D(double x1, double y1, double z1, double x2, double y2, double z2)
{
	glBegin(GL_LINES);
		glVertex3f(x1,y1,z1);
		glVertex3f(x2,y2,z2);
	glEnd();
}

////////////////////////////////////////////////////////////////////////////////
void drawSolidCircle(double x1, double y1, double radius_x, double radius_y)
{
	double radians;
	glBegin(GL_TRIANGLE_FAN);
 		glVertex2f(x1, y1);
 		for(int angle = 0; angle <= 360; angle+=5)
		{
			radians = M_PI*(double)angle/180;
 			glVertex2f(x1 + sin(radians) * radius_x, y1 + cos(radians) * radius_y);
		}
 	glEnd();
}

////////////////////////////////////////////////////////////////////////////////
void drawCircle(double x1, double y1,  double radius_x, double radius_y)
{
	double radians;
    const double factor = M_PI/180.0;
    
	glBegin(GL_LINE_LOOP);
 		for(int angle = 0; angle <= 360; angle+=5)
		{
			radians = factor * (double)angle;
 			glVertex2f(x1 + sin(radians) * radius_x, y1 + cos(radians) * radius_y);
		}
 	glEnd();
}

////////////////////////////////////////////////////////////////////////////////
// parallel to the x-z plane
void drawCircle3D(double x1, double y1,  double radius, double z)
{
	double radians;
	glBegin(GL_LINE_LOOP);
 		for(int angle = 0; angle <= 360; angle+=5)
		{
			radians = M_PI*(double)angle/180;
 			glVertex3f(x1 + sin(radians) * radius, y1 + cos(radians) * radius, z);
		}
 	glEnd();
}

////////////////////////////////////////////////////////////////////////////////
// Parallel to the x,y,z axis
void drawRect3D(double x1, double y1, double x2, double y2, double z)
{
    drawLine3D(x1,y1,z,x1,y2,z);
    drawLine3D(x1,y1,z,x2,y1,z);
    drawLine3D(x2,y2,z,x1,y2,z);
    drawLine3D(x2,y2,z,x2,y1,z);
}

////////////////////////////////////////////////////////////////////////////////
// Parallel to the x,y,z axis
void drawWiredBox(double x1, double y1, double z1, double x2, double y2, double z2)
{
    drawRect3D(x1,y1,x2,y2,z1);
    drawRect3D(x1,y1,x2,y2,z2);
    drawLine3D(x1,y1,z1,x1,y1,z2);
    drawLine3D(x2,y2,z1,x2,y2,z2);
    drawLine3D(x1,y2,z1,x1,y2,z2);
    drawLine3D(x2,y1,z1,x2,y1,z2);
}

////////////////////////////////////////////////////////////////////////////////
void drawRect( double x1, double y1, double x2, double y2)
{
    drawLine(x1,y1,x1,y2);
    drawLine(x1,y1,x2,y1);
    drawLine(x2,y2,x1,y2);
    drawLine(x2,y2,x2,y1);
}

#endif	/* GLSIMPLEOBJECTS_H */


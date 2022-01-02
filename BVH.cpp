#include <fstream>
#include <string>
#include <iostream>

#include "BVH.h"
// NOTE: This might not be needed
// #include "stdafx.h"

// Contractor
BVH::BVH() {
    motion = NULL;
    Clear();
}

// Contractor
BVH::BVH(const char* bvh_file_name) {
    motion = NULL;
    Clear();

    Load(bvh_file_name);
}

// Destructor
BVH::~BVH() { Clear(); }

// Clear all information
void BVH::Clear() {
    unsigned int i;
    for (i = 0; i < channels.size(); i++) delete channels[i];
    for (i = 0; i < joints.size(); i++) delete joints[i];
    if (motion != NULL) delete motion;

    is_load_success = false;

    file_name = "";
    motion_name = "";

    num_channel = 0;
    channels.clear();
    joints.clear();
    joint_index.clear();

    num_frame = 0;
    interval = 0.0;
    motion = NULL;
}

//
// Load BVH file
//
void BVH::Load(const char* bvh_file_name) {
#define BUFFER_LENGTH 1024 * 4

    ifstream file;
    char line[BUFFER_LENGTH];
    char* token;
    char separater[] = " :,\t";
    vector<Joint*> joint_stack;
    Joint* joint = NULL;
    Joint* new_joint = NULL;
    bool is_site = false;
    double x, y, z;
    int i, j;

    // Initialize
    Clear();

    // Setting file information (file name / operation name)
    file_name = bvh_file_name;
    const char* mn_first = bvh_file_name;
    const char* mn_last = bvh_file_name + strlen(bvh_file_name);
    if (strrchr(bvh_file_name, '\\') != NULL)
        mn_first = strrchr(bvh_file_name, '\\') + 1;
    else if (strrchr(bvh_file_name, '/') != NULL)
        mn_first = strrchr(bvh_file_name, '/') + 1;
    if (strrchr(bvh_file_name, '.') != NULL)
        mn_last = strrchr(bvh_file_name, '.');
    if (mn_last < mn_first) mn_last = bvh_file_name + strlen(bvh_file_name);
    motion_name.assign(mn_first, mn_last);

    // Open file
    file.open(bvh_file_name, ios::in);
    if (file.is_open() == 0) return;  // Exit if the file cannot be opened

    // Read hierarchical information
    while (!file.eof()) {
        // Abnormal termination when the end of the file is reached
        if (file.eof()) goto bvh_error;

        // Read one line and get the first word
        file.getline(line, BUFFER_LENGTH);
        token = strtok(line, separater);

        // If there is a blank line, go to the next line
        if (token == NULL) continue;

        // Start of joint block
        if (strcmp(token, "{") == 0) {
            // Stack the current joint
            joint_stack.push_back(joint);
            joint = new_joint;
            continue;
        }
        // End of joint block
        if (strcmp(token, "}") == 0) {
            // Remove the current joint from the stack
            joint = joint_stack.back();
            joint_stack.pop_back();
            is_site = false;
            continue;
        }

        // Start of joint information
        if ((strcmp(token, "ROOT") == 0) || (strcmp(token, "JOINT") == 0)) {
            // Create joint data
            new_joint = new Joint();
            new_joint->index = joints.size();
            new_joint->parent = joint;
            new_joint->has_site = false;
            new_joint->offset[0] = 0.0;
            new_joint->offset[1] = 0.0;
            new_joint->offset[2] = 0.0;
            new_joint->site[0] = 0.0;
            new_joint->site[1] = 0.0;
            new_joint->site[2] = 0.0;
            joints.push_back(new_joint);
            if (joint) joint->children.push_back(new_joint);

            // Read joint name
            token = strtok(NULL, "");
            while (*token == ' ') token++;
            new_joint->name = token;

            // Add to index
            joint_index[new_joint->name] = new_joint;
            continue;
        }

        // Start of end information
        if ((strcmp(token, "End") == 0)) {
            new_joint = joint;
            is_site = true;
            continue;
        }

        // Joint offset or end position information
        if (strcmp(token, "OFFSET") == 0) {
            // Read the coordinate values
            token = strtok(NULL, separater);
            x = token ? atof(token) : 0.0;
            token = strtok(NULL, separater);
            y = token ? atof(token) : 0.0;
            token = strtok(NULL, separater);
            z = token ? atof(token) : 0.0;
            // Set the coordinate value for the joint offset
            if (is_site) {
                joint->has_site = true;
                joint->site[0] = x;
                joint->site[1] = y;
                joint->site[2] = z;
            }
            else
                // Set the coordinate value at the end position
            {
                joint->offset[0] = x;
                joint->offset[1] = y;
                joint->offset[2] = z;
            }
            continue;
        }

        // Joint channel information
        if (strcmp(token, "CHANNELS") == 0) {
            // Read the number of channels
            token = strtok(NULL, separater);
            joint->channels.resize(token ? atoi(token) : 0);

            // Read channel information
            for (i = 0; i < joint->channels.size(); i++) {
                // Create channel
                Channel* channel = new Channel();
                channel->joint = joint;
                channel->index = channels.size();
                channels.push_back(channel);
                joint->channels[i] = channel;

                // Judgment of channel type
                token = strtok(NULL, separater);
                if (strcmp(token, "Xrotation") == 0)
                    channel->type = X_ROTATION;
                else if (strcmp(token, "Yrotation") == 0)
                    channel->type = Y_ROTATION;
                else if (strcmp(token, "Zrotation") == 0)
                    channel->type = Z_ROTATION;
                else if (strcmp(token, "Xposition") == 0)
                    channel->type = X_POSITION;
                else if (strcmp(token, "Yposition") == 0)
                    channel->type = Y_POSITION;
                else if (strcmp(token, "Zposition") == 0)
                    channel->type = Z_POSITION;
            }
        }

        // Move to the Motion data section
        if (strcmp(token, "MOTION") == 0) break;
    }

    // Load motion information
    file.getline(line, BUFFER_LENGTH);
    token = strtok(line, separater);
    if (strcmp(token, "Frames") != 0) goto bvh_error;
    token = strtok(NULL, separater);
    if (token == NULL) goto bvh_error;
    num_frame = atoi(token);

    file.getline(line, BUFFER_LENGTH);
    token = strtok(line, ":");
    if (strcmp(token, "Frame Time") != 0) goto bvh_error;
    token = strtok(NULL, separater);
    if (token == NULL) goto bvh_error;
    interval = atof(token);

    num_channel = channels.size();
    motion = new double[num_frame * num_channel];

    // Load motion data
    for (i = 0; i < num_frame; i++) {
        file.getline(line, BUFFER_LENGTH);
        token = strtok(line, separater);
        for (j = 0; j < num_channel; j++) {
            if (token == NULL) goto bvh_error;
            motion[i * num_channel + j] = atof(token);
            token = strtok(NULL, separater);
        }
    }

    // Close file
    file.close();

    // Successful loading
    is_load_success = true;

    return;

bvh_error:
    file.close();
}

//
// BVH skeleton / posture drawing function
//

#include <math.h>

#include "GL/glut.h"

// Draw the posture of the specified frame
void BVH::RenderFigure(int frame_no, float scale) {
    // Draw by specifying the BVH skeleton / posture
    RenderFigure(joints[0], motion + frame_no * num_channel, scale);
}

// Draw the specified BVH skeleton / posture (class function)
void BVH::RenderFigure(const Joint* joint, const double* data, float scale) {
    glPushMatrix();

    // Apply translation for root joints
    if (joint->parent == NULL) {
        //std::cout << "BVH: " << data[0]<< " " << data[1]<< " "
            //<< data[2]<< "\n";
        glTranslatef(data[0] * scale, data[1] * scale, data[2] * scale);
    }
    // For child joints, apply translation from the parent joint
    else {
        glTranslatef(joint->offset[0] * scale, joint->offset[1] * scale,
            joint->offset[2] * scale);
    }

    // Apply rotation from parent joint (rotation from world coordinates for root
    // joint)
    int i;
    for (i = 0; i < joint->channels.size(); i++) {
        Channel* channel = joint->channels[i];
        if (channel->type == X_ROTATION)
            glRotatef(data[channel->index], 1.0f, 0.0f, 0.0f);
        else if (channel->type == Y_ROTATION)
            glRotatef(data[channel->index], 0.0f, 1.0f, 0.0f);
        else if (channel->type == Z_ROTATION)
            glRotatef(data[channel->index], 0.0f, 0.0f, 1.0f);
    }

    // Draw a link
    // Draw a link from the origin to the end of the joint coordinate system
    if (joint->children.size() == 0) {
        RenderBone(0.0f, 0.0f, 0.0f, joint->site[0] * scale, joint->site[1] * scale,
            joint->site[2] * scale);
    }
    // Draw a link from the origin of the joint coordinate system to the
    // connection position to the next joint
    if (joint->children.size() == 1) {
        Joint* child = joint->children[0];
        RenderBone(0.0f, 0.0f, 0.0f, child->offset[0] * scale,
            child->offset[1] * scale, child->offset[2] * scale);
    }
    // Draw a cylinder from the center point to the connection position to all
    // joints to the connection position to each joint
    if (joint->children.size() > 1) {
        // Calculate the center point to the origin and the connection position to
        // all joints
        float center[3] = { 0.0f, 0.0f, 0.0f };
        for (i = 0; i < joint->children.size(); i++) {
            Joint* child = joint->children[i];
            center[0] += child->offset[0];
            center[1] += child->offset[1];
            center[2] += child->offset[2];
        }
        center[0] /= joint->children.size() + 1;
        center[1] /= joint->children.size() + 1;
        center[2] /= joint->children.size() + 1;

        // Draw a link from the origin to the center point
        RenderBone(0.0f, 0.0f, 0.0f, center[0] * scale, center[1] * scale,
            center[2] * scale);

        // Draw a link from the center point to the connection position to the next
        // joint
        for (i = 0; i < joint->children.size(); i++) {
            Joint* child = joint->children[i];
            RenderBone(center[0] * scale, center[1] * scale, center[2] * scale,
                child->offset[0] * scale, child->offset[1] * scale,
                child->offset[2] * scale);
        }
    }

    // Recursive call to the child joint
    for (i = 0; i < joint->children.size(); i++) {
        RenderFigure(joint->children[i], data, scale);
    }

    glPopMatrix();
}

// Draw a single link in the BVH skeleton (class function)
void BVH::RenderBone(float x0, float y0, float z0, float x1, float y1, float z1,
    float bRadius) {
    // Draw a cylinder connecting the given two points

    // Convert the information of the two end points of the cylinder to the
    // information of the origin, orientation, and length
    GLdouble dir_x = x1 - x0;
    GLdouble dir_y = y1 - y0;
    GLdouble dir_z = z1 - z0;
    GLdouble bone_length = sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);

    // Setting drawing parameters
    static GLUquadricObj* quad_obj = NULL;
    if (quad_obj == NULL) quad_obj = gluNewQuadric();
    gluQuadricDrawStyle(quad_obj, GLU_FILL);
    gluQuadricNormals(quad_obj, GLU_SMOOTH);

    glPushMatrix();

    // Set translation
    glTranslated(x0, y0, z0);

    // Below, calculate the matrix that represents the rotation of the cylinder

    // Normalize the z-axis to the unit vector
    double length;
    length = sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
    if (length < 0.0001) {
        dir_x = 0.0;
        dir_y = 0.0;
        dir_z = 1.0;
        length = 1.0;
    }
    dir_x /= length;
    dir_y /= length;
    dir_z /= length;

    // Set the reference y-axis orientation
    GLdouble up_x, up_y, up_z;
    up_x = 0.0;
    up_y = 1.0;
    up_z = 0.0;

    // Calculate the x-axis direction from the outer product of the z-axis and
    // y-axis
    double side_x, side_y, side_z;
    side_x = up_y * dir_z - up_z * dir_y;
    side_y = up_z * dir_x - up_x * dir_z;
    side_z = up_x * dir_y - up_y * dir_x;

    // Normalize x-axis to unit vector
    length = sqrt(side_x * side_x + side_y * side_y + side_z * side_z);
    if (length < 0.0001) {
        side_x = 1.0;
        side_y = 0.0;
        side_z = 0.0;
        length = 1.0;
    }
    side_x /= length;
    side_y /= length;
    side_z /= length;

    // Calculate the y-axis orientation from the outer product of the z-axis and
    // x-axis
    up_x = dir_y * side_z - dir_z * side_y;
    up_y = dir_z * side_x - dir_x * side_z;
    up_z = dir_x * side_y - dir_y * side_x;

    // Set the rotation matrix
    GLdouble m[16] = { side_x, side_y, side_z, 0.0, up_x, up_y, up_z, 0.0,
                      dir_x,  dir_y,  dir_z,  0.0, 0.0,  0.0,  0.0,  1.0 };
    glMultMatrixd(m);

    // Cylinder settings
    GLdouble radius = bRadius;  // Cylinder thickness
    GLdouble slices = 8.0;      // Radial fractions of the cylinder (default 12)
    GLdouble stack = 3.0;  // Subdivision of round slices of cylinder (default 1)

    // Draw a cylinder
    gluCylinder(quad_obj, radius, radius, bone_length, slices, stack);

    glPopMatrix();
}

void BVH::printTree(ofstream &f, Joint* root, std::string indent, bool last) 
{
    /*
    std::cout << indent << "+- " << root->name << "\n";;
    indent += last ? "  " : "| ";

    for (int i = 0; i < root->children.size(); i++)
    {
        printTree(root->children[i], indent, i == root->children.size() - 1);
    }*/

    std::vector<std::string> channelEnum = {
        "Xrotation",
        "Yrotation",
        "Zrotation",
        "Xposition",
        "Yposition",
        "Zposition" };

    int index = 0;

    if (root->parent == NULL)
    {
        f << "HIERARCHY" << "\n";
        f << "ROOT " << root->name << "\n";
        f << "{" << "\n";
        indent += "    ";
        f << indent << "OFFSET ";
        f << root->offset[0] << " ";
        f << root->offset[1] << " ";
        f << root->offset[2] << " ";
        f << "\n";
        f << indent << "CHANNELS ";
        f << root->channels.size() << " ";
        for (int i = 0; i < root->channels.size(); i++)
        {
            index = root->channels[i]->type;
            f << channelEnum[index] << " ";
        }
        f << "\n";
    }
    else
    {
        f << indent << "JOINT " << root->name << "\n";
        f << indent << "{" << "\n";
        indent += "    ";
        f << indent << "OFFSET ";
        f << root->offset[0] << " ";
        f << root->offset[1] << " ";
        f << root->offset[2] << " ";
        f << "\n";
        f << indent << "CHANNELS ";
        f << root->channels.size() << " ";
        for (int i = 0; i < root->channels.size(); i++)
        {
            index = root->channels[i]->type;
            f << channelEnum[index] << " ";
        }
        f << "\n";

        if (root->children.size() < 1)
        {
            f << indent << "End Site" << "\n";
            f << indent << "{" << "\n";
            f << indent << "    " << "OFFSET " << " ";
            f << 0.000000 << " ";
            f << 0.000000 << " ";
            f << 0.000000 << " ";
            f << "\n";
            //indent = indent.substr(0, indent.size() - 4);
            f << indent << "}" << "\n";
        }
    }

    for (int i = 0; i < root->children.size(); i++)
    {
        printTree(f, root->children[i], indent, i == root->children.size() - 1);
    }

    //indent = indent.substr(0, indent.size() - 4);
    for(int i = 0; i < 4; i++)
        indent.pop_back();
    
    f << indent << "}" << "\n";
}

// End of BVH.cpp
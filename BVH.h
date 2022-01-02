/**
*** BVH operation file reading / drawing class
*** Copyright (c) 2004-2007, Masaki OSHITA (www.oshita-lab.org)
**/

#pragma warning(disable : 4018 4996)
#ifndef _BVH_H_
#define _BVH_H_

#include <map>
#include <string>
#include <vector>

using namespace std;

//
// BVH format motion data
//
class BVH {
public:
    /* Internal structure */

    // Channel type
    enum ChannelEnum {
        X_ROTATION,
        Y_ROTATION,
        Z_ROTATION,
        X_POSITION,
        Y_POSITION,
        Z_POSITION
    };
    struct Joint;

    // Channel information
    struct Channel {
        // Corresponding joint
        Joint* joint;
        // Channel type
        ChannelEnum type;
        // Channel number
        int index;
    };

    // Joint information
    struct Joint {
        // Joint name
        string name;
        // Joint number
        int index;

        // Joint hierarchy (parent joint)
        Joint* parent;
        // Joint hierarchy (child joint)
        vector<Joint*> children;

        // Connection position
        double offset[3];

        // Flag to have end position information
        bool has_site;
        // Terminal position
        double site[3];

        // Axis of rotation
        vector<Channel*> channels;
    };

public:
    // Flag for successful load
    bool is_load_success;

    /* File information */
    string file_name;    // file name
    string motion_name;  // Action name


    /* Hierarchical information */
    int num_channel;             // number of channels
    vector<Channel*> channels;  // Channel information [channel number]
    vector<Joint*> joints;      // Joint information [part number]
    map<string, Joint*>
        joint_index;  // Index from joint name to joint information

    /* Motion data information */
    int num_frame;    // number of frames
    double interval;  // Time interval between frames
    double* motion;   // [frame number] [channel number]

public:
    // Constructor / Destructor
    BVH();
    BVH(const char* bvh_file_name);
    ~BVH();

    // Clear all information
    void Clear();

    // Load BVH file
    void Load(const char* bvh_file_name);

public:
    /* Data access function */

    // Get if the load was successful
    bool IsLoadSuccess() const { return is_load_success; }

    // Get file information
    const string& GetFileName() const { return file_name; }
    const string& GetMotionName() const { return motion_name; }

    // Get hierarchical structure information
    const int GetNumJoint() const { return joints.size(); }
    const Joint* GetJoint(int no) const { return joints[no]; }
    const int GetNumChannel() const { return channels.size(); }
    const Channel* GetChannel(int no) const { return channels[no]; }

    const Joint* GetJoint(const string& j) const {
        map<string, Joint*>::const_iterator i = joint_index.find(j);
        return (i != joint_index.end()) ? (*i).second : NULL;
    }
    const Joint* GetJoint(const char* j) const {
        map<string, Joint*>::const_iterator i = joint_index.find(j);
        return (i != joint_index.end()) ? (*i).second : NULL;
    }

    // Get motion data information
    int GetNumFrame() const { return num_frame; }
    double GetInterval() const { return interval; }
    double GetMotion(int f, int c) const { return motion[f * num_channel + c]; }

    // Change motion data information
    void SetMotion(int f, int c, double v) { motion[f * num_channel + c] = v; }

    void printTree(ofstream &f, Joint* root, std::string indent, bool last);

public:
    /* Posture drawing function */
    // Draw the posture of the specified frame
    void RenderFigure(int frame_no, float scale = 1.0f);

    // Draw the specified BVH skeleton / posture (class function)
    static void RenderFigure(const Joint* root, const double* data,
        float scale = 1.0f);

    // Draw a single link in the BVH skeleton (class function)
    static void RenderBone(float x0, float y0, float z0, float x1, float y1,
        float z1, float bRadius = 0.1);

};

#endif  // _BVH_H_
#ifndef __CAMERA_H
#define __CAMERA_H

#include <pthread.h>

class Camera {
public:
	Camera();


private:
	int processId_;
	pthread_t processThread_;

	static void *process_(void* cameraPtr);
};

#endif

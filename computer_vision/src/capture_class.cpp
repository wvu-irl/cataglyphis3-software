#include <computer_vision/capture_class.hpp>

Capture::Capture()
{
	camera_detection_error=0; 
}


void Capture::initialize_camera()
{
	//std::cout << "gp_camera_new (&camera) = " << gp_result_as_string(gp_camera_new (&camera)) << std::endl;
	gp_camera_new (&camera);
	context = gp_context_new();
	//gp_context_set_error_func(context, error_func, NULL);
	//gp_context_set_message_func(context, message_func, NULL);
}

void Capture::auto_detect_camera()
{
	ROS_WARN_ONCE("Camera inititialization can take more than 10 seconds depending on the memory card's contents (remove card from camera to speed up).");
	//This call will autodetect cameras, take the first one from the list and use it
	retval = gp_camera_init(camera, context);
	//std::cout << "gp_camera_init(camera, context) = " << gp_result_as_string(gp_camera_init(camera, context)) << std::endl;
	if (retval != GP_OK) 
	{
		ROS_ERROR_THROTTLE(2,"Camera auto detect failure. retval = %s", gp_result_as_string(retval));
		gp_camera_free(camera);
		//gp_camera_unref(camera);
		//gp_camera_exit(camera, context);
		camera_detection_error=1;
	}
	else
	{
		ROS_INFO("Camera auto detect success.");
		camera_detection_error=0;
	}
}

void Capture::capture_image_to_buffer()
{
	char *data;
	unsigned long size;
	
	capture_to_memory(camera, context, (const char**)&data, &size);
	std::vector<char> buff(data, data+size);
	image_UInt8MultiArray.data.clear();
	for(int i=0; i<buff.size(); i++)
	{
		image_UInt8MultiArray.data.push_back(buff[i]);
	}
	buff.clear();
}

void Capture::decode_image_from_buffer_to_mat()
{
	if(image_UInt8MultiArray.data.size()>0) 
	{
		image_Mat = imdecode(cv::Mat(image_UInt8MultiArray.data), 1);
		ROS_INFO("Decoding image...");
	}
	else
	{
		ROS_INFO("Buffer empty, image cannot be decoded.");
	}
}

int Capture::capture_success()
{
	if(image_Mat.empty()==1)
	{
		ROS_WARN("WARNING: Captured image empty!");
		return 0;
	}
	else
	{
		return 1;
	}
}

/*
void Capture::write_buffer_to_file(std::string filename)
{
	FILE *f;
	f = fopen(filename.c_str(), "wb");
	if(f)
	{
		retval=fwrite(data, size, 1, f);
		if(retval != size) { ROS_INFO("fwrite size %ld, written %d", size, retval); }
		fclose(f);
	}
	else
	{
		ROS_INFO("fwrite size %ld, written %d", size, retval);
	}
}
*/

void Capture::close_camera()
{
	gp_camera_exit(camera, context);
	ROS_INFO("Camera closed.");
}

/*
void Capture::error_func(GPContext *context, const char *format, va_list args, void *data) 
{
	fprintf  (stderr, "*** Contexterror ***\n");
	vfprintf (stderr, format, args);
	fprintf  (stderr, "\n");
}
*/

/*
void Capture::message_func(GPContext *context, const char *format, va_list args, void *data) 
{
	vprintf (format, args);
	printf ("\n");
}
*/

void Capture::capture_to_memory(Camera *camera, GPContext *context, const char **ptr, unsigned long int *size) 
{
	int retval;
	CameraFile *file;
	CameraFilePath camera_file_path;

	ROS_INFO("Capturing image to buffer...");

	/* NOP: This gets overridden in the library to /capt0000.jpg */
	strcpy(camera_file_path.folder, "/");
	strcpy(camera_file_path.name, "temp.jpg");

	retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
	if(retval !=0) ROS_INFO("gp_camera_capture retval: %d", retval);
	//ROS_INFO("Pathname on the camera: %s/%s", camera_file_path.folder, camera_file_path.name);

	retval = gp_file_new(&file);
	if(retval !=0) ROS_INFO("gp_file_new retval: %d", retval);

	retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
	if(retval !=0) ROS_INFO("gp_camera_file_get retval: %d", retval);

	gp_file_get_data_and_size (file, ptr, size);

	//ROS_INFO("Deleting.");
	retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name,
	context);
	if(retval !=0) ROS_INFO("gp_camera_file_delete retval: %d", retval);
}

void Capture::capture_to_file(Camera *camera, GPContext *context, char *fn)
{
	int fd, retval;
	CameraFile *file;
	CameraFilePath camera_file_path;

	ROS_INFO("Capturing image to file...");

	// NOP: This gets overridden in the library to /capt0000.jpg 
	strcpy(camera_file_path.folder, "/");
	strcpy(camera_file_path.name, "temp.jpg");

	retval = gp_camera_capture(camera, GP_CAPTURE_IMAGE, &camera_file_path, context);
	ROS_INFO("gp_camera_capture retval: %d", retval);
	ROS_INFO("Pathname on the camera: %s/%s", camera_file_path.folder, camera_file_path.name);

	fd = open(fn, O_CREAT | O_WRONLY, 0644);

	retval = gp_file_new_from_fd(&file, fd);
	ROS_INFO("gp_file_new_from_fd retval: %d", retval);

	retval = gp_camera_file_get(camera, camera_file_path.folder, camera_file_path.name, GP_FILE_TYPE_NORMAL, file, context);
	ROS_INFO("gp_camera_file_get retval: %d", retval);

	ROS_INFO("Deleting.");
	retval = gp_camera_file_delete(camera, camera_file_path.folder, camera_file_path.name, context);
	ROS_INFO("gp_camera_file_delete retval: %d", retval);

	gp_file_free(file);
}

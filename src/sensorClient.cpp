#include "flirBFS.h"
#include <boost/shared_ptr.hpp>
#include <boost/filesystem.hpp>
//#include <json.h>

#include <map>

//#include <b64/encode.h>
//#include <b64/decode.h>

#include "socketServer.h"

using namespace std;

// Serialize a cv::Mat to a stringstream
stringstream serialize(cv::Mat input)
{
    size_t size = input.total() * input.elemSize();

    stringstream ss;

    // Write the whole image data
    ss.write((char*)input.data, size);

    return ss;
}

// Function to compose the preview messages
/*
void composePreviewMsg(Json::Value& js_root, std::vector<cv::Mat>& vec_imgs)
{
    
    js_root["n_imgs"] = int(vec_imgs.size());
    js_root["imgs"] = Json::arrayValue;
    base64::encoder enc;
    for(cv::Mat img : vec_imgs)
    {
        // Serialize the input image to a stringstream
        Json::Value img_json;
        img_json["width"] = int(img.cols);
        img_json["height"] = int(img.rows);
        img_json["type"] = int(img.type());
        img_json["size"] = int(img.total() * img.elemSize());
        stringstream serializedStream = serialize(img);
        stringstream encoded;
        enc.encode(serializedStream, encoded);
        img_json["data"] = encoded.str();

        js_root["imgs"].append(img_json);
    }
    
}
*///

std::chrono::system_clock::time_point string_to_time_point(const std::string &str)
{
    using namespace std;
    using namespace std::chrono;

    int yyyy, mm, dd, HH, MM, SS, fff;

    char scanf_format[] = "%4d.%2d.%2d-%2d.%2d.%2d.%3d";

    sscanf(str.c_str(), scanf_format, &yyyy, &mm, &dd, &HH, &MM, &SS, &fff);

    tm ttm = tm();
    ttm.tm_year = yyyy - 1900; // Year since 1900
    ttm.tm_mon = mm - 1; // Month since January 
    ttm.tm_mday = dd; // Day of the month [1-31]
    ttm.tm_hour = HH; // Hour of the day [00-23]
    ttm.tm_min = MM;
    ttm.tm_sec = SS;

    time_t ttime_t = mktime(&ttm);

    system_clock::time_point time_point_result = std::chrono::system_clock::from_time_t(ttime_t);

    time_point_result += std::chrono::milliseconds(fff);
    return time_point_result;
}

std::string time_point_to_string(std::chrono::system_clock::time_point &tp)
{
    using namespace std;
    using namespace std::chrono;

    auto ttime_t = system_clock::to_time_t(tp);
    auto tp_sec = system_clock::from_time_t(ttime_t);
    milliseconds ms = duration_cast<milliseconds>(tp - tp_sec);

    std::tm * ttm = localtime(&ttime_t);

    char date_time_format[] = "%Y.%m.%d-%H.%M.%S";

    char time_str[] = "yyyy.mm.dd-HH.MM.SS.fff";

    strftime(time_str, strlen(time_str), date_time_format, ttm);

    string result(time_str);
    result.append(".");
    result.append(to_string(ms.count()));

    return result;
}


int main(int argc, char **argv)
{
    // Used by cameras
    std::map<string, std::map<string, string>> paras_cams;
    boost::shared_ptr<BFSMulti> pCameras(new BFSMulti()); 
    std::vector<boost::shared_ptr<BFS_camera>> cameras;
    // Retrieve reference to system object
    SystemPtr system = System::GetInstance();
    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

	unsigned int numCameras = camList.GetSize();

	cout << "Number of cameras detected: " << numCameras << endl << endl;
	
       // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        cout << "No enough cameras!" << endl;
		
		return 0;
    }
	
        // Init, connect to cameras
    for(int id = 0; id < numCameras; id++)
    {
        CameraPtr pCam = camList.GetByIndex(id);
    
        // Test the BFS camera class
        boost::shared_ptr<BFS_camera> pCamera(new BFS_camera(pCam));
        pCamera->cam_connect();

        // Get camera ID, check valid
        string camID = pCamera->cameraID;

		pCameras->BFS_new(pCamera, paras_cams[camID]);
		
		cout<<"connect to camera "<<camID<<endl;
    }
    std::cout<<"Cameras connected: " << pCameras->nCams_ <<std::endl;
            
    pCameras->BFSM_start_capturing();       
    
    int res; 
           
    //string str = "test"; 
    
    int count = 0;
    
    double initial_count = cv::getTickCount();
    
    Stime ts;
    
    while(1){
    
    	count++;
    	
    	int r_tg = pCameras->BFSM_trigger(ts);
    	
    	if(r_tg!=0)
            std::cout<<"Trigger Failed"<< std::endl;
            
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); //1000
        
        if(count>100)
        	break;
    }        
    //hardware
    //single camera: 12.1 FPS
    //TWO CAMERAS: 10.1 FPS
    //software 10FPS
    float dura = ((double)cv::getTickCount() - initial_count)/cv::getTickFrequency();
    std::cout<<"stop count "<<dura<<endl;
	cout<<"FPS: "<<count/dura<<endl;
	
    //stop trigger
    
    res = pCameras->BFSM_stop_trigger();
        
    //disconnect
    res = pCameras->BFSM_disconnect();

    camList.Clear();

    system->ReleaseInstance();  // Will report error here

    std::cout<<"Program end"<<std::endl; 
}


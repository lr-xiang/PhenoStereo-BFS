#include <sstream>
#include <chrono>
#include "flirBFS.h"
#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
#define DEBUG_INFO 1

// ************************* Black Fly S single camera control ***************************/

#define GPIO_PORT gpio256
//#define GPIO_PORT gpio397

BFS_camera::BFS_camera(CameraPtr p_cam):
    pCam(p_cam){
    
}

BFS_camera::~BFS_camera()
{
    if(_syncSave.csv_writter.is_open())
        _syncSave.csv_writter.close();
    cam_disconnect();
}

int BFS_camera::cam_connect()
{
    int result = 0;
    try
    {
        INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        // Retrieve TL device nodemap and print device information
        pDevNodeMap = &(pCam->GetTLDeviceNodeMap());

        if(DEBUG_INFO)
            result = PrintCameraInfo(*pDevNodeMap);
        
        CStringPtr pDevIdNode = pDevNodeMap->GetNode("DeviceID");
        
        cameraID = pDevIdNode->GetValue();

        // Initialize camera with sdk
        if(pCam->IsInitialized()){
            pCam->AcquisitionStop();
            pCam->DeInit();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        //pCam->EndAcquisition();
        pCam->Init();
        
        if(!pCam->IsInitialized()){
            std::cout<<"Camera cannot be initialized"<<std::endl;
            return -1;
        }

        // Retrieve GenICam nodemap
        pNodeMap = &(pCam->GetNodeMap());
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    isConnected = true;
    return result;
}

int BFS_camera::set_params(map<string, string> params)
{
    if(params.find("TriggerMode") != params.end())
        camParas.triggerMode = params["TriggerMode"];
    if(params.find("TriggerSource") != params.end())
        camParas.triggerSource = params["TriggerSource"];

    if(params.find("PixelFormat") != params.end())
        camParas.pixelFormat = params["PixelFormat"];

    if(params.find("ExposureAuto") != params.end())
        camParas.exposureAuto = params["ExposureAuto"];
    if(params.find("GainAuto") != params.end())
        camParas.gainAuto = params["GainAuto"]; 
    if(params.find("BalanceWhiteAuto") != params.end())
        camParas.balanceWhiteAuto = params["BalanceWhiteAuto"];

    if(params.find("ExposureTime") != params.end())
        camParas.exposureTime = stof(params["ExposureTime"]);

    if(params.find("Gain") != params.end())
        camParas.gain = stof(params["Gain"]);

    // If in streaming. If not streaming, no do start it.
    if(_streaming)
    {
        // First stop streaming, and EndAcquisition, then restart
        try{
            _streaming = false;
	        _threadWorker.join();
            cam_init();
        }
        catch (Spinnaker::Exception &e)
        {
            cout << "Error: " << e.what() << endl;
            return -1;
        }
    }

    return 0;
}

int BFS_camera::set_shutter(float shutter)
{
    try
    {
        if(set_node_val(pNodeMap, "ExposureTime", shutter)!=0)
        {
            std::cout<<"Set ExposureTime failed"<<std::endl;
            return -2;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

// Set parameters to camera, (re)start worker thread. 
int BFS_camera::cam_init()
{
    int result = 0;
    try{
        pCam->AcquisitionStop();
    }
    catch (Spinnaker::Exception &e)
    {
        cout<<"End acqusition: " << e.what() << endl;
    }
    std::clog<<"["<<cameraID<<"] Set parameters "<<std::endl;
    try
    {

        if(set_node_val(pNodeMap, "TriggerMode", camParas.triggerMode)!=0)
        {
            std::cout<<"Set TriggerMode failed"<<std::endl;
            return -1;
        }

        if(set_node_val(pNodeMap, "ExposureAuto", camParas.exposureAuto)!=0)
        {
            std::cout<<"Set ExposureAuto failed"<<std::endl;
            return -1;
        }
        if(set_node_val(pNodeMap, "BalanceWhiteAuto", camParas.balanceWhiteAuto)!=0)
        {
            std::cout<<"Set BalanceWhiteAuto failed"<<std::endl;
            return -1;
        }

        if(set_node_val(pNodeMap, "ExposureTime", camParas.exposureTime)!=0)
        {
            std::cout<<"Set ExposureTime failed"<<std::endl;
            return -1;
        }

        if(set_node_val(pNodeMap, "GainAuto", camParas.gainAuto)!=0)
        {
            std::cout<<"Set GainAuto failed"<<std::endl;
            return -1;
        }
    
        if(set_node_val(pNodeMap, "Gain", camParas.gain)!=0)
        {
            std::cout<<"Set Gain failed"<<std::endl;
            return -1;
        }

        if(set_node_val(pNodeMap, "TriggerSource", camParas.triggerSource)!=0)
        {
            std::cout<<"Set TriggerSource failed"<<std::endl;
            return -1;
        }

        std::cout<<"["<<cameraID<<"] " << "Trigger source: "<<camParas.triggerSource<<std::endl;

        if(camParas.triggerSource!="Software"){
            if(set_node_val(pNodeMap, "TriggerOverlap", "ReadOut")!=0)
            {
                std::cout<<"Set TriggerOverlap failed"<<std::endl;
                return -1;
            }
        }
        if(set_node_val(pNodeMap, "PixelFormat", camParas.pixelFormat)!=0)
        {
            std::cout<<"Set PixelFormat failed"<<std::endl;
            return -1;
        }

        start_streaming();

        start_saving();

    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

bool BFS_camera::cam_trigger_ready()
{
    return (!isTriggered) && _streaming && img_buffer_.size()<100;
}

void BFS_camera::cam_trigger(Stime& trigger_time)
{
    ptsTrigger = &trigger_time;
    if (camParas.triggerSource == "Software")
    {
        // Execute software trigger
        CCommandPtr ptrSoftwareTriggerCommand = pNodeMap->GetNode("TriggerSoftware");
        if (!IsAvailable(ptrSoftwareTriggerCommand) || !IsWritable(ptrSoftwareTriggerCommand))
        {
            cout << "Unable to execute trigger. Aborting..." << endl;
        }
        
        ptrSoftwareTriggerCommand->Execute();

        trigger_time = std::chrono::system_clock::now();
    }
    
    isTriggered = true;
}

int BFS_camera::cam_disconnect()
{
    std::clog<<"["<<cameraID<<"] Camera disconnecting "<<std::endl;
    
    if(_streaming){
        try{
            _streaming = false;
	        _threadWorker.join();
        }
        catch (Spinnaker::Exception &e)
        {
            cout << "Error: " << e.what() << endl;
            return -1;
        }
    }
    if(is_saving_){
        try{
            is_saving_ = false;
	        thread_saver_.join();
        }
        catch (Spinnaker::Exception &e)
        {
            cout << "Error: " << e.what() << endl;
            return -1;
        }
    }

    isConnected = false;
    if(pCam!=0)
        pCam->DeInit();
    pCam = 0;
    std::clog<<"["<<cameraID<<"] Camera disconnected "<<std::endl;
	return 0;
}

int BFS_camera::cam_get_img(cv::Mat& mat_out)
{
    try{
        _updateMutex.lock();
        mat_out = cvImage.clone();
        _updateMutex.unlock();
    }
    catch(std::exception& e)
    {
        std::cout<<"Get image error: "<<e.what()<<std::endl;
        return -1;
    }
    return 0;
}

// Streaming

void BFS_camera::start_streaming()
{
    
    // Start worker process to receive images
    
	_threadWorker = std::thread(&BFS_camera::streaming, this);
    cout << "[" << cameraID << "] " << "Started acquiring images..." << endl;
}

void BFS_camera::streaming()
{
    // Start camera streaming, if using continues mode
    pCam->BeginAcquisition();
    // image id counter reset
    imgId = 0;

    _streaming = true;
    // Worker thread main loop
    while((_streaming && isConnected))
    {
        if(!isTriggered){
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        if(DEBUG_INFO)
            cout << "[" << cameraID << "] "<< ". Grab image " << endl;
        try
        {
            // Retrieve next received image and ensure image completion
            imgId ++; 
            pImage = pCam->GetNextImage();
            
            // Check the triggerMode and is module active
            // If passive mode, or trigger Off, save all image with received time as timestamp
            // If active mode, and trigger On, save the triggered image, and trigger timestamp
            Stime triggered_time;
            if(camParas.triggerMode == "Off") {
                triggered_time = std::chrono::system_clock::now();
            }
            else{
                triggered_time = *ptsTrigger;
                isTriggered = false;
            }

            if (pImage->IsIncomplete())
            {
                cout << "[" << cameraID << "] " << "Image incomplete with image status " << pImage->GetImageStatus() << "..." << endl << endl;
            }
            else
            {
                // Convert image (TODO: check format)
                ImagePtr convertedImage = pImage;//pImage->Convert(PixelFormat_BayerRG8, HQ_LINEAR);
                // Save images to buffer                
                unsigned int XPadding = convertedImage->GetXPadding();
                unsigned int YPadding = convertedImage->GetYPadding();
                unsigned int rowsize = convertedImage->GetWidth();
                unsigned int colsize = convertedImage->GetHeight();

                // image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
                // Copy data to shared buffer
                _updateMutex.lock();
                cvImage = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, convertedImage->GetData(), convertedImage->GetStride()).clone();
                _updateMutex.unlock();

                // Save the image, with filename&timestamp
                if(1){
                    // Put the image into a saving buffer, leave the rest of the saving work to a saving thread
                    _updateMutex.lock();

                    img_buffer_.push_back(std::pair<int, cv::Mat>(
                            _syncSave.calc_stime_ms_diff(triggered_time),
                            cvImage.clone()));
                    _updateMutex.unlock();
                    // Migrated these to the saving thread    
                    /* 
                    // Create a unique filename
                    std::stringstream ss_fn; // A string stream to generate filename, with format
                    ss_fn<<std::setw(5)<<std::setfill('0')<<_syncSave.save_id<<"_"<<cameraID<<".jpeg";
                    fs::path img_path = fs::path(_syncSave.save_path) / fs::path(ss_fn.str());

                    // Save image
                    //convertedImage->Save(img_path.c_str());

                    // Print image information
                    if(DEBUG_INFO)
                    cout << "[" << cameraID << "] "<< ". Image saved at " << img_path.c_str()<< endl;
                    _syncSave.save_id++;

                    // Log file with timestamp. 
                    if(_syncSave.csv_writter.is_open())
                        _syncSave.csv_writter<<
                            _syncSave.calc_stime_ms_diff(triggered_time)
                            <<","<<img_path.c_str()<<std::endl;*/
                }
            }
        
            // Release image
            pImage->Release();
            
            
        }
        catch (Spinnaker::Exception &e)
        {
            cout << "[" << cameraID << "] " << "Error: " << e.what() << endl;
        }
        
    }
    cout<<"[" << cameraID << "] " << "Stoped streaming loop"<<std::endl;
    pCam->EndAcquisition();
    cout<<"[" << cameraID << "] " << "Stoped streaming"<<std::endl;
}

// Saving threads related

void BFS_camera::start_saving()
{ 
    // Start worker process to receive images
    
	thread_saver_ = std::thread(&BFS_camera::saving, this);
}

void BFS_camera::saving()
{
    cout << "[" << cameraID << "] " << "Started saving thread." << endl;
    is_saving_ = true;

    vector<int> compression_params;
#if CV_VERSION_MAJOR == 3
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
#else
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
#endif
    compression_params.push_back(85);

    while(is_saving_ && isConnected)
    {

        while(!img_buffer_.empty())
        {
            // Create a unique filename
            std::stringstream ss_fn; // A string stream to generate filename, with format

            ss_fn<<std::setw(5)<<std::setfill('0')<<_syncSave.save_id<<"_"<<cameraID<<".jpeg";
            fs::path img_path = fs::path(_syncSave.save_path) / fs::path(ss_fn.str());

            // Print image information
            if(DEBUG_INFO)
                cout << "[" << cameraID << "] "<< ". Image saved at " << img_path.c_str()<< endl;
            _syncSave.save_id++;

            // Save image
            _updateMutex.lock();

            std::pair<int, cv::Mat> ts_img = img_buffer_.front();
            cv::Mat rgb_converted;
#if CV_VERSION_MAJOR == 3
            cv::cvtColor(ts_img.second, rgb_converted, cv::COLOR_BayerRG2RGB);
#else
            cv::cvtColor(ts_img.second, rgb_converted, CV_BayerRG2RGB);
#endif
            cv::imwrite(img_path.c_str(), rgb_converted, compression_params);

            img_buffer_.pop_front();
            _updateMutex.unlock();

            // Log file with timestamp. 
            if(_syncSave.csv_writter.is_open())
                _syncSave.csv_writter<<
                    ts_img.first
                    <<","<<img_path.c_str()<<std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    img_buffer_.clear();
    cout << "[" << cameraID << "] " <<"Saving thread stoped"<<std::endl;
}

// Nodes

int BFS_camera::set_node_val(INodeMap* p_node_map, string node_name, string value)
{
    int result = 0;
    try{
        CValuePtr p_val = p_node_map->GetNode(node_name.c_str());
        if (!IsAvailable(p_val) || !IsWritable(p_val))
        {
            cout << "Unable set str Node "<<node_name;
            if(!IsAvailable(p_val))
                cout<<" Not available";
            if(!IsWritable(p_val))
                cout<<" Not writable";
            cout <<". Aborting..." << endl << endl;
            return -1;
        }
        // Enum type
        CEnumEntryPtr p_enumVal = ((CEnumerationPtr)p_val)->GetEntryByName(value.c_str());
        if (!IsAvailable(p_enumVal) || !IsReadable(p_enumVal))
        {
            cout << "Unable to set Enum node: "<< node_name << ", with value: " << value <<". Aborting..." << endl << endl;
            return -1;
        }
        ((CEnumerationPtr)p_val)->SetIntValue(p_enumVal->GetValue());
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}
int BFS_camera::set_node_val(INodeMap* p_node_map, string node_name, int value)
{
    int result = 0;
    try{
        CValuePtr p_val = p_node_map->GetNode(node_name.c_str());
        if (!IsAvailable(p_val) || !IsWritable(p_val))
        {
            cout << "Unable set int Node "<<node_name<<". Aborting..." << endl << endl;
            return -1;
        }
        ((CIntegerPtr)p_val)->SetValue(value);
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}
int BFS_camera::set_node_val(INodeMap* p_node_map, string node_name, float value)
{
    int result = 0;
    try{
        CValuePtr p_val = p_node_map->GetNode(node_name.c_str());
        if (!IsAvailable(p_val) || !IsWritable(p_val))
        {
            cout << "Unable set float Node "<<node_name<<". Aborting..." << endl << endl;
            return -1;
        }
        ((CFloatPtr)p_val)->SetValue(value); 
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}
template<typename T>
int BFS_camera::get_node_val(INodeMap* p_node_map, string node_name, T& value)
{
    int result = 0;
    try{
        CValuePtr p_val = p_node_map->GetNode(node_name.c_str());
        if (!IsAvailable(p_val) || !IsReadable(p_val))
        {
            cout << "Unable get Node "<<node_name<<". Aborting..." << endl << endl;
            return -1;
        }
        // String type
        if(is_same<T, string>::value){   
            value = ((CStringPtr)p_val)->GetValue();
        }
        // Int type
        else if(is_same<T, int>::value){
            value = ((CIntegerPtr)p_val)->GetValue();
        }
        // Float type
        else if(is_same<T, float>::value || is_same<T, double>::value)
        {
            value = ((CFloatPtr)p_val)->GetValue();
        }
        else{
            std::cout<<"Set value datatype not implemented"<<std::endl;
            return -1;
        }
        
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}

// ************************* Black Fly S multiple camera control ***************************/

BFSMulti::BFSMulti()
{
    ts_buffer_.resize(15);
    gpioExport(GPIO_PORT);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    gpioSetDirection(GPIO_PORT, outputPin);
    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

BFSMulti::~BFSMulti()
{
    
    int res = BFSM_disconnect();
    if(res!=0)
        std::cout<<"Disconnect error: "<< -res << std::endl;
}

// A new camera to organize together, with its parameters
void BFSMulti::BFS_new(ptrCamera pcamera, std::map<string, string>& paras_cam)
{
    cameras_.push_back(pcamera);
    pcamera->set_params(paras_cam);
    nCams_ += 1;
}

int BFSMulti::BFSM_disconnect()
{
    // Stop triggering thread
    cont_triggering_ = false;
    if(thread_trigger_.joinable())
        thread_trigger_.join();

    // Stop & release cameras
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->cam_disconnect();
        if(res!=0)
            return -(std::stoi(pCamera->cameraID));
    }
    cameras_.clear();
    nCams_ = 0;
    
    // Clear buffers
    ts_buffer_.clear();
    std::clog<<"All disconnected "<<std::endl;
    return 0;
}

// Syncronization
void BFSMulti::BFSM_sync(Stime ts)
{
    for(ptrCamera pCamera : cameras_)
    {
        std::cout<<"["<<pCamera->cameraID<<"] "<<"Sync"<<std::endl;
        pCamera->setup_sync(ts);
    }
}

// Set parameters
int BFSMulti::BFSM_set_param(std::map<string, std::map<string, string>>& paras_cams, std::string save_path){
    for(ptrCamera pCamera : cameras_)
    {
        //std::cout<<"["<<pCamera->cameraID<<"] "<<"Paras"<<std::endl;
        int res = pCamera->set_params(paras_cams[pCamera->cameraID]);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
        //std::cout<<"["<<pCamera->cameraID<<"] "<<"Save"<<std::endl;
        pCamera->setup_save(true, pCamera->cameraID, save_path);

        // Working mode, "Active", "Passive"
        if(moduleMode_ == "Client")
            pCamera->is_active_mode_ = false;
        else
            pCamera->is_active_mode_ = true;
    }
    return 0;
}

// Set exposure (shutter)
int BFSMulti::BFSM_set_exposure(int t_us){
    std::cout<<"Set exposure to: "<<t_us<<" us" << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_shutter(t_us);
        //pCamera->set_shutter(t_us);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

// Start capturing
int BFSMulti::BFSM_start_capturing(){
    for(ptrCamera pCamera : cameras_)
    {
        std::cout<<"["<<pCamera->cameraID<<"] "<<"Capture"<<std::endl;
        int res = pCamera->cam_init();
        if(res!=0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

// Trigger camera
int BFSMulti::BFSM_trigger(Stime& ts){  
    
    // wait for trigger ready
    int try_time_out = 0;
    bool trigger_ready = true;
    while(try_time_out<2000){
        trigger_ready = true;
        for(ptrCamera pCamera : cameras_)
        {
            trigger_ready = trigger_ready && pCamera->cam_trigger_ready();
        }
        if(trigger_ready)
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        try_time_out ++;
    }
    
    // Time out for triggering
    if(!trigger_ready){
    	cout<<"trigger is not ready\n";
        return -1;
	}
    // Trigger the cameras
    // If the module needs to trigger any cameras, save the actual triggering time
    if(moduleMode_ == "Soft" || moduleMode_ == "GPIO"){
        ts = std::chrono::system_clock::now();
        // GPIO trigger
        if(moduleMode_ == "GPIO"){
            gpioSetValue(GPIO_PORT, on);
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
            gpioSetValue(GPIO_PORT, off);
        }
            
    }
    // Trigger cameras
    for(ptrCamera pCamera : cameras_)
    {
        pCamera->cam_trigger(ts);
    }
    return 0;
}

// Auto trigger, 
int BFSMulti::BFSM_auto_trigger(int frames, float freq){

    if(cont_triggering_){
        std::cout<<"Already triggering, operation aborted"<<std::endl;
        return -1;    
    }
    if(thread_trigger_.joinable())
        thread_trigger_.join();

    cont_triggering_ = true;

    // TODO: set this for different triggering modes
    float trigger_cost_ms;
    if(moduleMode_ == "Soft") 
        trigger_cost_ms = 22.2; 
    else if (moduleMode_ == "GPIO")
        trigger_cost_ms = 45.5;
    else
        trigger_cost_ms = 45.5;

    int ms_sleep = MAX(int(1000.0/freq - trigger_cost_ms), 0);

    std::cout<<"Start continuous triggering"<<std::endl;
    thread_trigger_ = std::thread(
            // Triggering threads body as a Lambda function
            [this, frames, ms_sleep]()
            {
                int frames_taken = 0;
                Stime t_start = std::chrono::system_clock::now();
                for (int i = 0; i < frames || frames < 0; i++){
                    // Save time and trigger
                    ts_buffer_.push_back(std::chrono::system_clock::now());
                    int r_tg = BFSM_trigger(ts_buffer_.back());
                    // Check success
                    if(r_tg==0)
                        std::cout<<"Trigger"<<std::endl;
                    else
                    {
                        std::cout<<"Trigger failed"<< std::endl;
                        cont_triggering_ = false;
                        return;
                    }
                    // Wait to achieve different frame rates
                    std::this_thread::sleep_for(std::chrono::milliseconds(ms_sleep));
                
                    frames_taken++;

                    // Check if stoped by other threads
                    if(!cont_triggering_)
                        break;
                }
                cont_triggering_ = false;
                int ms_used = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-t_start).count();

                cout<<"Frame rate:"<<1000.0 * frames_taken / ms_used<<" Hz"<<std::endl;
            }
    );
    return 0;
}

int BFSMulti::BFSM_stop_trigger()
{
    std::cout<<"Stop triggering"<<std::endl;
    cont_triggering_ = false;
    if(cont_triggering_){
        if(thread_trigger_.joinable())
            thread_trigger_.join(); 
        else
            cout<<"Triggering thread not joinable"<<endl;  
    }
    return 0;
}


// Get previews
int BFSMulti::BFSM_get_previews(std::vector<cv::Mat>& vec_imgs)
{
    vec_imgs.resize(nCams_);
    
    for(int i = 0; i < nCams_; i++)
    {
        int r_tg = cameras_[i]->cam_get_img(vec_imgs[i]);
        if(r_tg!=0)
        {
            std::cout<<"Get images "<< cameras_[i]->cameraID <<" failed"<< std::endl;
            return -1;
        }
    }
    return 0;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintCameraInfo(INodeMap & nodeMap)
{
    int result = 0;

    cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

    try
    {
        FeatureList_t features;
        CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            FeatureList_t::const_iterator it;
            for (it = features.begin(); it != features.end(); ++it)
            {
                CNodePtr pfeatureNode = *it;
                cout << pfeatureNode->GetName() << " : ";
                CValuePtr pValue = (CValuePtr)pfeatureNode;
                cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                cout << endl;
            }
        }
        else
        {
            cout << "Device control information not available." << endl;
        }
    }
    catch (Spinnaker::Exception &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include "nlohmann/json.hpp"
#include <experimental/filesystem>
#include <fstream>
#include <unistd.h>
#include "Playback_Framework/ConfigMsg.h"

namespace fs = std::experimental::filesystem;
Playback_Framework::ConfigMsg readconfigfile(const char * readerpath)
{
  Playback_Framework::ConfigMsg data;
  std::ifstream file(readerpath);
  if(!file){
       std::cout << "Failed to open file" << std::endl;
  }
  nlohmann::json newJson = nlohmann::json::parse(file);
  data.Mode=newJson["RosbagManager"]["Mode"];
  std::cout<<"Mode:"<<data.Mode<<std::endl; 
 
  data.modeparameter=newJson["RosbagManager"]["ModeParameter"];
  std::cout<<"Modeparameter:"<<data.modeparameter<<std::endl;
 
  data.Path=newJson["RosbagManager"]["Bag"];
  std::cout<<"Path:"<<data.Path<<std::endl;

  data.Windowsize=newJson["RosbagManager"]["Window"];
  std::cout<<"Windowsize:"<<data.Windowsize<<std::endl;

  data.delay=newJson["RosbagManager"]["Delay"];
  std::cout<<"Delay:"<<data.delay<<std::endl;

  for (auto& rosbagtopic : newJson["RosbagManager"]["RosbagTopic"].items())
  {
        data.RosbagTopics.push_back(rosbagtopic.value());       
  }
  for (auto& el : newJson["RosbagManager"]["Topics"].items())
  {
	auto e2 = el.value();
	for(auto& e3 : e2.items())
	{
       	   data.Topics.push_back(e3.value());
	   //std::cout << e3.key()<<"     "<<e3.value()<< '\n';
	  //std::cout << newJson["RosbagManager"]["Topics"][0]["cam_topic"]<< '\n';
	}  
  }
  for(int i=0;i<data.Topics.size();i++)
  {
    std::cout <<data.Topics[i] << '\n';
  }


return data;
}

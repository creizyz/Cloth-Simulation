#include "Utils.h"

#include<fstream>
#include<iostream>

const std::string utils::readFile(const std::string & path)
{
  std::string buffer;
  std::ifstream ifs;
  ifs.open(path, std::ifstream::in);
  if (!ifs.is_open())
  {
    std::cout << "file at \"" << path << "\" could not be open" << std::endl;
  }
  else
  {
    std::getline(ifs, buffer, (char)EOF);
    ifs.close();
  }
  return buffer;
}
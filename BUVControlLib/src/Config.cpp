#include "Config.hpp"

#include <fstream>
#include <iostream>


Config::Config(std::string const& filename) {
	std::ifstream file(filename.c_str()); // ler ficheiro

	std::string line;
	std::string name;
	std::string value;
	int posSpace;
	while (std::getline(file,line)) { //leitura de uma linha do ficheiro e fica guardado na variavel line

		if (! line.length()) continue; //ignorar linhas vazias

		if (line[0] == '#') continue; //definir quando é usado # é comantario e é ignorada
		
		if (line[line.length()-1] == '\r') {
			line = line.substr(0, line.length()-1);
		}

		posSpace = line.find(' '); // improve to allow any type of space including \t ? definir que depois do espaço vem o valor da variavel 
		name  = line.substr(0,posSpace);
		value = line.substr(posSpace+1);

		content[name] = value;
	}
}

std::string Config::getString(std::string const& name, std::string const& defaultValue) {
	std::map<std::string,std::string>::const_iterator ci = content.find(name);

	if (ci == content.end()) 
		return defaultValue;

	return ci->second;
}

float Config::getFloat(std::string const& name, float defaultValue) {
	std::map<std::string,std::string>::const_iterator ci = content.find(name);

	if (ci == content.end()) 
		return defaultValue;

	std::string::size_type sz;
	return std::stof(ci->second,&sz);
}

bool Config::getBool(std::string const& name, bool defaultValue) {
	std::map<std::string,std::string>::const_iterator ci = content.find(name);

	if (ci == content.end()) 
		return defaultValue;
	int i = std::stoi(ci->second);
	assert(i == 0  || i == 1);
  	return i == 1;
}

Eigen::Vector3f Config::getVector3f(std::string const& name, Eigen::Vector3f defaultValue) {
	Eigen::Vector3f v;
	std::map<std::string,std::string>::const_iterator ci = content.find(name);

	if (ci == content.end()) 
		return defaultValue;

	std::string::size_type sz;
	float e1 = std::stof(ci->second,&sz);
	std::string::size_type sz2;
	float e2 = std::stof(ci->second.substr(sz),&sz2);
	float e3 = std::stof(ci->second.substr(sz + sz2),&sz);

	v << e1, e2, e3;
	return v;
}

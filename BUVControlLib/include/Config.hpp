#ifndef __CONFIG_FILE_H__
#define __CONFIG_FILE_H__

#include <Eigen/Dense>
#include <string>
#include <map>

class Config {
public:
	Config(std::string const& filename);

	std::string getString(std::string const& name, std::string const& defaultValue);
	float getFloat(std::string const& name, float defaultValue);
	bool getBool(std::string const& name, bool defaultValue);
	Eigen::Vector3f getVector3f(std::string const& name, Eigen::Vector3f defaultValue);

protected:
	std::map<std::string, std::string> content;
};

#endif

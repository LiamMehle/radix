#ifndef SHADER_H
#define SHADER_H

#include <glm/glm.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class Shader {
public:
    unsigned int ID;

    // Constructor: compiles shaders from provided file paths
    Shader(const char* vertexPath, const char* fragmentPath, const char* geometryPath = nullptr) {
        std::string vertexCode = readShaderCode(vertexPath);
        std::string fragmentCode = readShaderCode(fragmentPath);
        std::string geometryCode = geometryPath ? readShaderCode(geometryPath) : "";

        unsigned int vertex = compileShader(vertexCode.c_str(), GL_VERTEX_SHADER, "VERTEX");
        unsigned int fragment = compileShader(fragmentCode.c_str(), GL_FRAGMENT_SHADER, "FRAGMENT");
        unsigned int geometry = geometryPath ? compileShader(geometryCode.c_str(), GL_GEOMETRY_SHADER, "GEOMETRY") : 0;

        // Link shaders into a program
        ID = glCreateProgram();
        glAttachShader(ID, vertex);
        glAttachShader(ID, fragment);
        if (geometryPath) glAttachShader(ID, geometry);
        linkProgram(ID);

        // Cleanup
        glDeleteShader(vertex);
        glDeleteShader(fragment);
        if (geometryPath) glDeleteShader(geometry);
    }

    // Activate the shader program
    void use() { glUseProgram(ID); }

    // Uniform utility functions
    void setBool(const std::string &name, bool value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), static_cast<int>(value)); 
    }

    void setInt(const std::string &name, int value) const {
        glUniform1i(glGetUniformLocation(ID, name.c_str()), value); 
    }

    void setFloat(const std::string &name, float value) const {
        glUniform1f(glGetUniformLocation(ID, name.c_str()), value); 
    }

private:
    // Reads shader code from a file
    std::string readShaderCode(const char* shaderPath) {
        std::ifstream shaderFile(shaderPath);
        std::stringstream shaderStream;
        if (shaderFile) {
            shaderStream << shaderFile.rdbuf();
            shaderFile.close();
            return shaderStream.str();
        } else {
            std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << shaderPath << std::endl;
            return "";
        }
    }

    // Compiles a shader and checks for errors
    unsigned int compileShader(const char* shaderCode, GLenum shaderType, const std::string& type) {
        unsigned int shader = glCreateShader(shaderType);
        glShaderSource(shader, 1, &shaderCode, NULL);
        glCompileShader(shader);
        checkCompileErrors(shader, type);
        return shader;
    }

    // Links shaders into a program and checks for errors
    void linkProgram(unsigned int programID) {
        glLinkProgram(programID);
        checkCompileErrors(programID, "PROGRAM");
    }

    // Checks shader and program compilation/linking errors
    void checkCompileErrors(GLuint shader, const std::string& type) {
        GLint success;
        GLchar infoLog[1024];
        if(type != "PROGRAM") {
            glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
            if(!success) {
                glGetShaderInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        } else {
            glGetProgramiv(shader, GL_LINK_STATUS, &success);
            if(!success) {
                glGetProgramInfoLog(shader, 1024, NULL, infoLog);
                std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
            }
        }
    }
};

#endif // SHADER_H

#pragma once
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <sstream>
#include <fstream>
#include <future>
#include <iostream>

#define GLM_ENABLE_EXPERIMENTAL
#define GLM_FORCE_XYZW_ONLY 1
#include <glm/vec2.hpp> // glm::vec3
#include <glm/vec3.hpp> // glm::vec3
#include <glm/vec4.hpp> // glm::vec4
#include <glm/mat4x4.hpp> // glm::mat4
#include <glm/ext/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale
#include <glm/ext/scalar_constants.hpp> // glm::pi<float>()
#include <glm/gtc/quaternion.hpp> // glm::mat4
#include "glmext/aabox2.h"
#include "glmext/aabox.h"
#include "glmext/Plane.h"
#include "glmext/Units.h"
#include "glmext/Sphere.h"
#include "glmext/Ray.h"


typedef unsigned char byte;

#include "MyProject.hpp"
#define GLM_FORCE_RADIANS
#define FLM_FORCE_DEPTH_ZERO_TO_ONE

#define _USE_MATH_DEFINES
#include <math.h>

float scalingFactor = 40.0f;
static float rocketResizeFactor = 0.18;
static glm::vec3 startPoint = glm::vec3(0, 4.1085 * rocketResizeFactor + 0.7, 0);

struct GlobalUniformBufferObject {
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
};

//set 1 change per objcet
struct UniformBufferObject {
	alignas(16) glm::mat4 model;
};


class MyProject : public BaseProject {
	protected:
	
	// Descriptor Layouts [what will be passed to the shaders]
	DescriptorSetLayout DSLGlobal;
	DescriptorSetLayout DSLObject;
	DescriptorSetLayout DSLSkybox;

	// Pipelines [Shader couples]
	Pipeline P1;
	Pipeline SkyboxPipeline;

	// Models, textures and Descriptors (values assigned to the uniforms)
	Model Rocket_body;
	Texture Rocket_texture;
	DescriptorSet DS_Rocket;

	Model Landscape;
	Texture Landscape_texture;
	DescriptorSet DS_Landscape;
	
	Model Skybox;
	Texture Skybox_texture;
	DescriptorSet DS_Skybox;

	DescriptorSet DS_Global;
	
	// Here you set the main application parameters
	void setWindowParameters() {
		// window size, titile and initial background
		windowWidth = 800;
		windowHeight = 600;
		windowTitle = "Rocket Simulator";
		initialBackgroundColor = {1.0f, 0.65f, 0.4f, 1.0f};
		
		// Descriptor pool sizes
		uniformBlocksInPool = 8;
		texturesInPool = 8;
		setsInPool = 8;
	}

	void createDescriptorSetLayouts() {
		// Descriptor Layouts [what will be passed to the shaders]
		DSLGlobal.init(this, {
			// this array contains the binding:
			// first  element : the binding number
			// second element : the time of element (buffer or texture)
			// third  element : the pipeline stage where it will be used
			{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_ALL_GRAPHICS}
			});

		DSLObject.init(this, {
					{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT},
					{1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT}
			});

		DSLSkybox.init(this, {
					{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT},
					{1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT}
			});
	}

	void createPipelines() {
		// Pipelines [Shader couples]
		// The last array, is a vector of pointer to the layouts of the sets that will
		// be used in this pipeline. The first element will be set 0, and so on..
		P1.init(this, "shaders/vert.spv", "shaders/frag.spv", { &DSLGlobal, &DSLObject });
		//this bool is used to allow the usa of the same function but with some different parameters
		isSkybox = true;
		SkyboxPipeline.init(this, "shaders/SkyBoxVert.spv", "shaders/SkyBoxFrag.spv", { &DSLSkybox });
		isSkybox = false;
	}
	
	// Here you load and setup all your Vulkan objects
	void localInit() {
		// Models, textures and Descriptors (values assigned to the uniforms)
		Rocket_body.init(this, "models/R27-Ready.obj");
		Rocket_texture.init(this, "textures/R27 Texture.png");
		DS_Rocket.init(this, &DSLObject, {
		// the second parameter, is a pointer to the Uniform Set Layout of this set
		// the last parameter is an array, with one element per binding of the set.
		// first  elmenet : the binding number
		// second element : UNIFORM or TEXTURE (an enum) depending on the type
		// third  element : only for UNIFORMs, the size of the corresponding C++ object
		// fourth element : only for TEXTUREs, the pointer to the corresponding texture object
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &Rocket_texture}
				});

		Landscape.init(this, "models/desert2.obj");
		Landscape_texture.init(this, "textures/sand.png");
		DS_Landscape.init(this, &DSLObject, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &Landscape_texture}
				});

		DS_Global.init(this, &DSLGlobal, {
					{0, UNIFORM, sizeof(GlobalUniformBufferObject), nullptr},
				});

		//skybox model texture and DS
		isSkybox = true;
		Skybox.init(this, "models/SkyBoxCube.obj");
		Skybox_texture.init(this, "Skybox");	//the string is actually useless since I have an array of strings with all the 6 faces
		DS_Skybox.init(this, &DSLObject, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &Skybox_texture}
				});
		isSkybox = false;
	}

	//useful to separate the recreation of the descriptor sets when the window size is modified
	void descriptorSetsRecreation() {
		DS_Rocket.init(this, &DSLObject, {
						{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
						{1, TEXTURE, 0, &Rocket_texture}
			});

		DS_Landscape.init(this, &DSLObject, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &Landscape_texture}
			});

		DS_Global.init(this, &DSLGlobal, {
					{0, UNIFORM, sizeof(GlobalUniformBufferObject), nullptr},
			});

		DS_Skybox.init(this, &DSLObject, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &Skybox_texture}
			});
	}

	//cleanup divided so that I can manage better what to clean in case of a window resize, since those objects can remain the same while other must be recreated
	void objectsCleanup() {
		Rocket_texture.cleanup();
		Rocket_body.cleanup();

		Landscape_texture.cleanup();
		Landscape.cleanup();

		Skybox_texture.cleanup();
		Skybox.cleanup();

		DSLSkybox.cleanup();
		DSLGlobal.cleanup();
		DSLObject.cleanup();
	}

	void pipelineCleanup() {
		SkyboxPipeline.cleanup();
		P1.cleanup();
	}

	void descriptorSetsCleanup() {
		DS_Rocket.cleanup();
		DS_Landscape.cleanup();
		DS_Global.cleanup();
		DS_Skybox.cleanup();
	}
	
	// Here it is the creation of the command buffer: send to the GPU all the objects you want to draw, with their buffers and textures
	void populateCommandBuffer(VkCommandBuffer commandBuffer, int currentImage) {
				
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
				P1.graphicsPipeline);
		vkCmdBindDescriptorSets(commandBuffer,
			VK_PIPELINE_BIND_POINT_GRAPHICS,
			P1.pipelineLayout, 0, 1, &DS_Global.descriptorSets[currentImage],
			0, nullptr);
				
		VkBuffer vertexBuffers[] = { Rocket_body.vertexBuffer};
		// property .vertexBuffer of models, contains the VkBuffer handle to its vertex buffer
		VkDeviceSize offsets[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
		// property .indexBuffer of models, contains the VkBuffer handle to its index buffer
		vkCmdBindIndexBuffer(commandBuffer, Rocket_body.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		// property .pipelineLayout of a pipeline contains its layout.
		// property .descriptorSets of a descriptor set contains its elements.
		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 1, 1, &DS_Rocket.descriptorSets[currentImage],
						0, nullptr);
						
		// property .indices.size() of models, contains the number of triangles * 3 of the mesh.
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(Rocket_body.indices.size()), 1, 0, 0, 0);
	
		//Landscape part, maybe let's find better names
		VkBuffer vertexBuffers2[] = { Landscape.vertexBuffer };
		VkDeviceSize offsets2[] = { 0 };
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers2, offsets2);
		vkCmdBindIndexBuffer(commandBuffer, Landscape.indexBuffer, 0,
VK_INDEX_TYPE_UINT32);
vkCmdBindDescriptorSets(commandBuffer,
	VK_PIPELINE_BIND_POINT_GRAPHICS,
	P1.pipelineLayout, 1, 1, &DS_Landscape.descriptorSets[currentImage],
	0, nullptr);
vkCmdDrawIndexed(commandBuffer,
	static_cast<uint32_t>(Landscape.indices.size()), 1, 0, 0, 0);

//skybox
vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, SkyboxPipeline.graphicsPipeline);
VkBuffer vertexBuffers6[] = { Skybox.vertexBuffer };
VkDeviceSize offsets6[] = { 0 };
vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers6, offsets6);
vkCmdBindIndexBuffer(commandBuffer, Skybox.indexBuffer, 0, VK_INDEX_TYPE_UINT32);
vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
	SkyboxPipeline.pipelineLayout, 0, 1, &DS_Skybox.descriptorSets[currentImage],
	0, nullptr);
vkCmdDrawIndexed(commandBuffer, static_cast<uint32_t>(Skybox.indices.size()), 1, 0, 0, 0);
	}

	void firstPersonCamera(GlobalUniformBufferObject* gubo, UniformBufferObject* ubo, float lastTime, float deltaT, int width, int height) {
		//save pos when switch cam and then do difference between the movement of first and second camera
		static glm::mat3 CamDir = glm::mat3(glm::rotate(glm::mat4(1), -glm::radians(30.0f), glm::vec3(1, 0, 0)));
		static glm::vec3 CamPos = glm::vec3(0.0f, 9.0f, 9.0f);

		// rotation through mouse
		float cameraRotSpeed = 0.003f;

		// for unlimited movement in the window
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		// raw mouse input (no acceleration from the OS)
		if (glfwRawMouseMotionSupported())
			glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

		static double oldxpos = 0, oldypos = 0;
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		if (lastTime - deltaT == 0.0f) {
			oldxpos = xpos;
			oldypos = ypos;
		}

		glm::vec2 mouseMovement = glm::vec2(xpos - oldxpos, ypos - oldypos);
		//if (mouseMovement.x != 0) std::cout << mouseMovement.x << std::endl;

		if (sqrt(mouseMovement.x * mouseMovement.x + mouseMovement.y * mouseMovement.y) < 150) {
			CamDir = glm::mat3((glm::rotate(glm::mat4(1.0f), -mouseMovement.x * cameraRotSpeed, glm::vec3(0, 1, 0))) * glm::mat4(CamDir));
			CamDir = glm::mat3((glm::rotate(glm::mat4(1.0f), -mouseMovement.y * cameraRotSpeed, glm::vec3(CamDir[0]))) * glm::mat4(CamDir));
		}

		oldxpos = xpos;
		oldypos = ypos;

		// camera movement
		static glm::vec3 cameraPos = glm::vec3(0, 0, 0);
		float cameraSpeed = 0.9f; // units/second
		float yCameraSpeed = 2.0f;
		glm::vec3 oldCamPos = CamPos;

		if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)) {
			cameraSpeed *= 3.5;
			yCameraSpeed *= 3.5;
		}
		if (glfwGetKey(window, GLFW_KEY_A)) {
			CamPos -= cameraSpeed * deltaT * glm::vec3(CamDir[0]);
		}
		if (glfwGetKey(window, GLFW_KEY_D)) {
			CamPos += cameraSpeed * deltaT * glm::vec3(CamDir[0]);
		}
		if (glfwGetKey(window, GLFW_KEY_S)) {
			CamPos += cameraSpeed * deltaT * glm::vec3(glm::cross(CamDir[0], glm::vec3(0, 1, 0)));
		}
		if (glfwGetKey(window, GLFW_KEY_W)) {
			CamPos -= cameraSpeed * deltaT * glm::vec3(glm::cross(CamDir[0], glm::vec3(0, 1, 0)));
		}
		if (glfwGetKey(window, GLFW_KEY_C)) {
			if (CamPos.y > 2) {
				CamPos -= yCameraSpeed * deltaT * glm::vec3(0, 1, 0);
			}
		}
		if (glfwGetKey(window, GLFW_KEY_SPACE)) {
			CamPos += yCameraSpeed * deltaT * glm::vec3(0, 1, 0);
		}

		if (!canStep(CamPos)) {
			CamPos = oldCamPos;
		}

		gubo->view = glm::translate(glm::transpose(glm::mat4(CamDir)), -CamPos);
		gubo->proj = glm::perspective(glm::radians(45.0f),
			width / (float)height,
			0.1f, 40.0f);
		gubo->proj[1][1] *= -1;

	}

	bool canStep(glm::vec3 Camera) {
		float camDistance = ((Camera.x - startPoint.x) * (Camera.x - startPoint.x)) + ((Camera.z - startPoint.z) * (Camera.z - startPoint.z));
		//check the distance of the cam from the rocket, if camera closer than a variable return false so there's no clipping
		if (camDistance <= ( rocketResizeFactor * rocketResizeFactor) && Camera.y <= 2.3) {
				return false;
			}
		//std:: cout << Camera.y << std::endl;
		return true;
	}

	void topViewCamera(GlobalUniformBufferObject* gubo, UniformBufferObject* ubo, glm::vec3 *destPoint, int width, int height) {
		static glm::mat3 CamDir = glm::mat3(glm::rotate(glm::mat4(1), -glm::radians(90.0f), glm::vec3(1, 0, 0)));
		static glm::vec3 CamPos = glm::vec3(0.0f, 70.0f, 0.0f);

		gubo->view = glm::translate(glm::transpose(glm::mat4(CamDir)), -CamPos);
		gubo->proj = glm::scale(glm::mat4(1), glm::vec3(1, -1, 1)) *
			glm::ortho(CamPos.x - width / scalingFactor,
				CamPos.x + width / scalingFactor,
				CamPos.z + height / scalingFactor,
				CamPos.z - height / scalingFactor,
				0.1f, 75.0f);
		gubo->proj[1][1] *= -1;
	}

	void rocketPOVCamera(GlobalUniformBufferObject* gubo, UniformBufferObject* ubo, glm::vec3 rocketPos, glm::mat4 rocketRot, glm::vec3 trajDir, double beta, float ellipticTrajectoryFactor, int width, int height) {
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		
		//set the destination point first at random else 0

		// position of the camera relative to the rocket center
		float rotAngle = atan(ellipticTrajectoryFactor * cos(beta) / sin(beta)) + M_PI / 2;
		glm::vec3 camRelativePos = glm::vec3(-trajDir.x * cos(rotAngle), sin(rotAngle), -trajDir.z * cos(rotAngle)) * 0.3f;		

		glm::mat3 CamDir = glm::mat3(glm::rotate(glm::mat4(1), -glm::radians(70.0f) + rotAngle, glm::cross(glm::vec3(0, 1, 0), trajDir)) *
			glm::rotate(glm::mat4(1), (float) M_PI + glm::atan(trajDir.x, trajDir.z), glm::vec3(0, 1, 0)));
		glm::vec3 CamPos = rocketPos + camRelativePos;


		gubo->view = glm::translate(glm::transpose(glm::mat4(CamDir)), -CamPos);
		gubo->proj = glm::perspective(glm::radians(45.0f),
			width / (float)height,
			0.1f, 40.0f);
		gubo->proj[1][1] *= -1;
	}


	// Here is where you update the uniforms.
	// Very likely this will be where you will be writing the logic of your application.
	void updateUniformBuffer(uint32_t currentImage) {
		static auto startTime = std::chrono::high_resolution_clock::now();
		static float lastTime = 0.0f;
		auto currentTime = std::chrono::high_resolution_clock::now();
		float time = std::chrono::duration<float, std::chrono::seconds::period>
			(currentTime - startTime).count();
		float deltaT = time - lastTime;
		lastTime = time;

		GlobalUniformBufferObject gubo{};
		UniformBufferObject ubo{};
		void* data;

		// get window size
		int width, height;
		glfwGetWindowSize(window, &width, &height);

		// rocket
		//start point temporarely put in the global variables so i can access it from every method even in the hpp
		static glm::vec3 destPoint = glm::vec3(5, 4.1085 * rocketResizeFactor, 5);

		// rocket movement
		glm::vec3 center = glm::vec3((destPoint.x + startPoint.x) / 2, std::max(startPoint.y, destPoint.y), (destPoint.z + startPoint.z) / 2);
		float r = glm::sqrt(pow(destPoint.x - startPoint.x, 2) + pow(destPoint.z - startPoint.z, 2)) / 2;
		//std::cout << r << std::endl;

		static glm::vec3 rocketMov = glm::vec3(0);
		glm::mat4 rocketRot = glm::mat4(1);

		glm::vec3 diff = glm::normalize(glm::vec3(destPoint.x - startPoint.x, 0, destPoint.z - startPoint.z));
		float alpha = glm::atan(diff.z, diff.x);
		static double beta = M_PI;

		static float maxSpeed = 4.0f;
		static float acceleration = 2.0f;
		static float currentSpeed = 0.0f;
		float ellipticTrajectoryFactor = 2.0f;

		static bool launched = false;
		if (glfwGetKey(window, GLFW_KEY_ENTER)) {
			if (!launched) {
				launched = true;
			}
		}

		if (launched) {
			float movDelta = currentSpeed * deltaT;
			if (beta >= M_PI && startPoint.y + rocketMov.y + movDelta < destPoint.y) {
				rocketMov += glm::vec3(0, movDelta, 0);
			}
			else if (beta <= 0 && startPoint.y + rocketMov.y - movDelta > destPoint.y) {
				rocketMov -= glm::vec3(0, movDelta, 0);
				rocketRot = glm::rotate(glm::mat4(1), (float)M_PI, glm::cross(glm::vec3(0, 1, 0), diff));
			}
			else {
				rocketMov = glm::vec3(center.x - startPoint.x + r * cos(beta) * cos(alpha),
					center.y - startPoint.y + r * ellipticTrajectoryFactor * sin(beta),
					center.z - startPoint.z + r * cos(beta) * sin(alpha));
				float rotAngle = atan(ellipticTrajectoryFactor * cos(beta) / sin(beta)) + M_PI / 2;
				rocketRot = glm::rotate(glm::mat4(1), rotAngle, glm::cross(glm::vec3(0, 1, 0), diff));
				beta -= 2 * asin(movDelta / (2 * r));
			}

			if (beta <= 0 && startPoint.y + rocketMov.y - movDelta <= destPoint.y) {
				launched = false;
				currentSpeed = 0.0f;
				rocketMov = glm::vec3(0);
				rocketRot = glm::mat4(1);
				beta = M_PI;
				startPoint = destPoint;
				//change temporarely the destpoint in order not to get a blank visual during the RocketPOV 
				destPoint = glm::vec3(0, 4.1085 * rocketResizeFactor, 0);
			}
			if (currentSpeed < maxSpeed) {
				currentSpeed += acceleration * deltaT;
			}
		}

		// reset rocket position
		if (glfwGetKey(window, GLFW_KEY_R)) {
			launched = false;
			currentSpeed = 0.0f;
			rocketMov = glm::vec3(0);
			rocketRot = glm::mat4(1);
			beta = M_PI;
		}

		// camera
		static int cameraType = 0;
		float minInputDelay = 0.2f;
		static float timer = minInputDelay;
		timer += deltaT;
		if (glfwGetKey(window, GLFW_KEY_M) && timer >= minInputDelay) {
			cameraType = (cameraType + 1) % 3;
			timer = 0.0f;
		}

		switch (cameraType) {
		case 0:
			firstPersonCamera(&gubo, &ubo, lastTime, deltaT, width, height);
			break;
		case 1:
			topViewCamera(&gubo, &ubo, &destPoint, width, height);

			// change destination point on click when camera is from above
			glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			if (glfwGetKey(window, GLFW_KEY_X) && !launched) {
				double xpos, zpos;
				glfwGetCursorPos(window, &xpos, &zpos);
				destPoint.x = xpos / (scalingFactor / 2) - width / scalingFactor;
				destPoint.z = zpos / (scalingFactor / 2) - height / scalingFactor;
				//clipping y
				destPoint.y = 4.1085 * rocketResizeFactor + Landscape.findClosestPoint(destPoint.x, destPoint.z);
			}
			break;
		case 2:
			rocketPOVCamera(&gubo, &ubo, startPoint + rocketMov, rocketRot, diff, beta, ellipticTrajectoryFactor, width, height);
			break;
		}


		//Global shader
		vkMapMemory(device, DS_Global.uniformBuffersMemory[0][currentImage], 0,
			sizeof(gubo), 0, &data);
		memcpy(data, &gubo, sizeof(gubo));
		vkUnmapMemory(device, DS_Global.uniformBuffersMemory[0][currentImage]);

		// Rocket body
		ubo.model = glm::translate(glm::mat4(1), rocketMov)
			* glm::translate(glm::mat4(1), startPoint)
			* rocketRot
			* glm::rotate(glm::mat4(1), -glm::radians(90.0f), glm::vec3(1, 0, 0))
			* glm::scale(glm::mat4(1.0f), glm::vec3(rocketResizeFactor));
		vkMapMemory(device, DS_Rocket.uniformBuffersMemory[0][currentImage], 0,
			sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DS_Rocket.uniformBuffersMemory[0][currentImage]);

		//Landscape positioning
		glm::mat4 rot, trans, scale;
		rot = rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		trans = translate(glm::mat4(1.0f), glm::vec3(0, 0, 0));
		scale = glm::scale(glm::mat4(1), glm::vec3(6.5, 1, 6.5));
		ubo.model = trans * scale;
		vkMapMemory(device, DS_Landscape.uniformBuffersMemory[0][currentImage], 0,
			sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DS_Landscape.uniformBuffersMemory[0][currentImage]);
		Landscape.fillVectorY(ubo.model);

		//update skybox uniforms
		ubo.model = gubo.proj * glm::mat4(glm::mat3(gubo.view));
		vkMapMemory(device, DS_Skybox.uniformBuffersMemory[0][currentImage], 0,
			sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DS_Skybox.uniformBuffersMemory[0][currentImage]);
	}
};

// This is the main: probably you do not need to touch this!
int main() {
    MyProject app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
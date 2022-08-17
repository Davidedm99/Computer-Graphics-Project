// This has been adapted from the Vulkan tutorial

#include "MyProject.hpp"
#define GLM_FORCE_RADIANS
#define FLM_FORCE_DEPTH_ZERO_TO_ONE

#define _USE_MATH_DEFINES
#include <math.h>



// The uniform buffer object 

struct GlobalUniformBufferObject {
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
};

//set 1 change per objcet
struct UniformBufferObject {
	alignas(16) glm::mat4 model;
};




// MAIN ! 
class MyProject : public BaseProject {
	protected:
	// Here you list all the Vulkan objects you need:
	
	// Descriptor Layouts [what will be passed to the shaders]
	DescriptorSetLayout DSLGlobal;
	DescriptorSetLayout DSLObject;

	// Pipelines [Shader couples]
	Pipeline P1;

	// Models, textures and Descriptors (values assigned to the uniforms)
	Model Rocket_body;
	Texture Rocket_texture;
	DescriptorSet DS_Rocket;

	Model Landscape;
	Texture Landscape_texture;
	DescriptorSet DS_Landscape;

	DescriptorSet DS_Global;
	
	// Here you set the main application parameters
	void setWindowParameters() {
		// window size, titile and initial background
		windowWidth = 800;
		windowHeight = 600;
		windowTitle = "Rocket Simulator";
		initialBackgroundColor = {0.0f, 1.0f, 1.0f, 1.0f};
		
		// Descriptor pool sizes
		uniformBlocksInPool = 3;
		texturesInPool = 2;
		setsInPool = 3;
	}
	
	// Here you load and setup all your Vulkan objects
	void localInit() {
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

		// Pipelines [Shader couples]
		// The last array, is a vector of pointer to the layouts of the sets that will
		// be used in this pipeline. The first element will be set 0, and so on..
		P1.init(this, "shaders/vert.spv", "shaders/frag.spv", {&DSLGlobal, &DSLObject});

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

		Landscape.init(this, "models/mount.obj");
		Landscape_texture.init(this, "textures/ground.jpg");
		DS_Landscape.init(this, &DSLObject, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &Landscape_texture}
				});

		DS_Global.init(this, &DSLGlobal, {
					{0, UNIFORM, sizeof(GlobalUniformBufferObject), nullptr},
				});
	}

	// Here you destroy all the objects you created!		
	void localCleanup() {
		DS_Rocket.cleanup();
		Rocket_texture.cleanup();
		Rocket_body.cleanup();

		DS_Landscape.cleanup();
		Landscape_texture.cleanup();
		Landscape.cleanup();

		DS_Global.cleanup();

		P1.cleanup();
		DSLGlobal.cleanup();
		DSLObject.cleanup();
	}
	
	// Here it is the creation of the command buffer:
	// You send to the GPU all the objects you want to draw,
	// with their buffers and textures
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


		GlobalUniformBufferObject gubo{};
		UniformBufferObject ubo{};
		void* data;


		// camera
		static glm::mat3 CamDir = glm::mat3(glm::rotate(glm::mat4(1), -glm::radians(30.0f), glm::vec3(1, 0, 0))) * glm::mat3(1);
		static glm::vec3 CamPos = glm::vec3(0.0f, 9.0f, 9.0f);


		// rotation through mouse
		float cameraRotSpeed = 0.003f;

		// for unlimited movement in the window
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

		if (glfwRawMouseMotionSupported())
			glfwSetInputMode(window, GLFW_RAW_MOUSE_MOTION, GLFW_TRUE);

		static double oldxpos = 0, oldypos = 0;
		double xpos, ypos;
		glfwGetCursorPos(window, &xpos, &ypos);
		if (lastTime == 0.0f) {
			oldxpos = xpos;
			oldypos = ypos;
		}

		lastTime = time;

		glm::vec2 mouseMovement = glm::vec2(xpos - oldxpos, ypos - oldypos);

		CamDir = glm::mat3((glm::rotate(glm::mat4(1.0f), -mouseMovement.x * cameraRotSpeed, glm::vec3(0, 1, 0))) * glm::mat4(CamDir));
		CamDir = glm::mat3((glm::rotate(glm::mat4(1.0f), -mouseMovement.y * cameraRotSpeed, glm::vec3(CamDir[0]))) * glm::mat4(CamDir));

		oldxpos = xpos;
		oldypos = ypos;

		// camera movement
		static glm::vec3 cameraPos = glm::vec3(0, 0, 0);
		float cameraSpeed = 0.9f; // units/second
		float yCameraSpeed = 2.0f;
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
			CamPos -= yCameraSpeed * deltaT * glm::vec3(0, 1, 0);
		}
		if (glfwGetKey(window, GLFW_KEY_SPACE)) {
			CamPos += yCameraSpeed * deltaT * glm::vec3(0, 1, 0);
		}


		gubo.view = glm::translate(glm::transpose(glm::mat4(CamDir)), -CamPos);
		gubo.proj = glm::perspective(glm::radians(45.0f),
			swapChainExtent.width / (float)swapChainExtent.height,
			0.1f, 40.0f);
		gubo.proj[1][1] *= -1;


		// rocket movement test
		glm::vec3 startPoint = glm::vec3(0, 4.1085 * 0.3, 0);
		glm::vec3 destPoint = glm::vec3(5, 4.1085 * 0.3, 5);
		glm::vec3 center = glm::vec3((destPoint.x + startPoint.x) / 2, std::max(startPoint.y, destPoint.y), (destPoint.z + startPoint.z) / 2);
		float r = glm::sqrt(pow(destPoint.x - startPoint.x, 2) + pow(destPoint.z - startPoint.z, 2)) / 2;

		static glm::vec3 rocketMov = glm::vec3(0);
		glm::mat4 rocketRot = glm::mat4(1);

		glm::vec3 diff = destPoint - startPoint;
		float alpha = glm::atan(diff.z, diff.x);
		static double beta = M_PI;

		float maxSpeed = 15.0f;
		float acceleration = 6.0f;
		static float currentSpeed = 0.0f;

		static bool launched = false;
		static bool inputLimit = true;
		if (glfwGetKey(window, GLFW_KEY_ENTER)) {
			if (inputLimit) {
				launched = true;
				inputLimit = false;
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
				rocketMov = glm::vec3(center.x + r * cos(beta) * cos(alpha),
					center.y - startPoint.y + r * sin(beta),
					center.z + r * cos(beta) * sin(alpha));
				rocketRot = glm::rotate(glm::mat4(1), (float)(M_PI - beta), glm::cross(glm::vec3(0, 1, 0), diff));
				beta -= 2 * asin(movDelta / (2 * r));
			}

			if (beta <= 0 && startPoint.y + rocketMov.y - movDelta <= destPoint.y) {
				launched = false;
			}
			if (currentSpeed < maxSpeed) {
				currentSpeed += acceleration * deltaT;
			}
		}


		if (glfwGetKey(window, GLFW_KEY_R)) {
			launched = false;
			inputLimit = true;
			currentSpeed = 0.0f;
			rocketMov = glm::vec3(0);
			beta = M_PI;
		}


		//Global shader
		vkMapMemory(device, DS_Global.uniformBuffersMemory[0][currentImage], 0,
			sizeof(gubo), 0, &data);
		memcpy(data, &gubo, sizeof(gubo));
		vkUnmapMemory(device, DS_Global.uniformBuffersMemory[0][currentImage]);

		// Rocket body
		// rocket center position (0,0,4.1085)
		ubo.model = glm::translate(glm::mat4(1), rocketMov)
			* glm::translate(glm::mat4(1), startPoint)
			* rocketRot
			* glm::rotate(glm::mat4(1), -glm::radians(90.0f), glm::vec3(1, 0, 0))
			* glm::scale(glm::mat4(1.0f), glm::vec3(0.3f, 0.3f, 0.3f));
		vkMapMemory(device, DS_Rocket.uniformBuffersMemory[0][currentImage], 0,
			sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DS_Rocket.uniformBuffersMemory[0][currentImage]);

		//Landscape positioning
		glm::mat4 rot, trans, scale;
		rot = rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f));
		trans = translate(glm::mat4(1.0f), glm::vec3(-3, 0, -2));
		scale = glm::scale(glm::mat4(1), glm::vec3(3, 1, 3));
		ubo.model = trans * scale;
		vkMapMemory(device, DS_Landscape.uniformBuffersMemory[0][currentImage], 0,
			sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DS_Landscape.uniformBuffersMemory[0][currentImage]);
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
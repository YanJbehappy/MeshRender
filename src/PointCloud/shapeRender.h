/**
 * @description: 线、面等元素的渲染
 * @author YanJ
 * @date 2022/03/03 12:30
 * @version  
 */
#pragma once

#include <iostream>
#include <memory>
#include <QOpenGlFunctions_4_5_Core>
#include <QStack>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Base/shader.h>
#include <Base/texture.h>
#include <Base/framebuffer.h>
#include <Base/glbuffer.h>
#include <Base/camera.h>
#include <Base/glutils.h>

#include <PointCloud/pointclouds.h>
#include <PointCloud/uploader.h>


class RenderWidget;

class ShapeRender : protected QOpenGLFunctions_4_5_Core
{
public:
	ShapeRender(RenderWidget* glWidget, GLenum renderType);
	~ShapeRender();

	// 输入顶点数据
	void setInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, int r, int g, int b, int size);
	// 控制是否进行渲染
	void setVisible(bool visible) { this->visible_ = visible; }
	// 形状绘制函数
	void renderShape();

private:
	void init();

private:
	RenderWidget* glWidget_ = nullptr;
	Camera* camera_ = nullptr;
	Shader* shapeVBOShader_ = nullptr;		// 线面着色器
	GLBuffer* shapeBuffer_ = nullptr;		// 形状缓存

	GLenum renderType_;						// 记录渲染类型
	bool visible_;
	Eigen::MatrixXf data_;					// 记录顶点坐标颜色信息
	int pointNums_;							// 记录顶点数量
	int shapeSize_;							// 记录顶点尺寸

};

/**
 * @description: 线、面等元素的渲染
 * @author YanJ
 * @date 2022/03/03 12:30
 * @version
 */
#include "PointCloud/shapeRender.h"
#include <PointCloud/renderingWidget.h>


ShapeRender::ShapeRender(RenderWidget* glWidget, GLenum renderType)
	: visible_(true)
	, pointNums_(0)
	, shapeSize_(0)
{
	initializeOpenGLFunctions();
	this->glWidget_ = glWidget;
	this->camera_ = glWidget->camera;
	this->shapeVBOShader_ = glWidget->shapeVBOShader;

	this->renderType_ = renderType;

	glWidget->makeCurrent();

}

ShapeRender::~ShapeRender()
{
	glDeleteBuffers(1, &shapeBuffer_->VBO);
	glDeleteVertexArrays(1, &shapeBuffer_->VAO);
	delete shapeBuffer_;
}

void ShapeRender::setInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, int r, int g, int b, int size)
{
	this->pointNums_ = points->size();
	this->shapeSize_ = size;

	data_.resize(6, pointNums_);
	int rgb = r << 16 | g << 8 | b;

#pragma omp parallel for
	for (int i = 0; i < pointNums_; i++)
	{
		Eigen::Matrix<float, 6, 1> curVertex;
		curVertex(0, 0) = points->points[i].x;
		curVertex(1, 0) = points->points[i].y;
		curVertex(2, 0) = points->points[i].z;
		curVertex(3, 0) = r / 255.0;
		curVertex(4, 0) = g / 255.0;
		curVertex(5, 0) = b / 255.0;
		
		data_.col(i) = curVertex;
	}

	init();
}

void ShapeRender::init()
{
	vector<GLBufferAttribute> shapeAttributes;
	GLBufferAttribute pcdAttribute1("aPosition", 0, 3, GL_FLOAT, GL_FALSE, 24, 0); // xyz 12字节
	shapeAttributes.emplace_back(pcdAttribute1);
	GLBufferAttribute pcdAttribute2("aColor", 1, 3, GL_FLOAT, GL_FALSE, 24, 12); // rgb 12字节
	shapeAttributes.emplace_back(pcdAttribute2);

	shapeBuffer_ = new GLBuffer();
	shapeBuffer_->set(data_.data(), 24 * pointNums_, shapeAttributes, pointNums_);// 每个点三个坐标值一个颜色值共16byte
}

void ShapeRender::renderShape()
{
	if (!visible_)
	{
		return;
	}

	glDisable(GL_DEPTH_TEST);
	shapeVBOShader_->bind();

	Eigen::Matrix4f transform = camera_->getTransform();

	shapeVBOShader_->setUniformValue("uWorldViewProj", transform);
	shapeVBOShader_->setUniformValue("uSize", shapeSize_);
	glBindVertexArray(shapeBuffer_->VAO);

	switch (renderType_)
	{
	case GL_POINTS:
		glDrawArrays(GL_POINTS, 0, pointNums_);
		break;
	case GL_LINES:
		glLineWidth(3.0);
		glDrawArrays(GL_LINES, 0, pointNums_);
		break;
	case GL_LINE_STRIP:
		break;
	case GL_TRIANGLES:
		break;
	default:
		break;
	}

	glBindVertexArray(0);

	shapeVBOShader_->unbind();

	glEnable(GL_DEPTH_TEST);

}


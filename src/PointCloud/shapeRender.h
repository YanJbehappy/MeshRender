/**
 * @description: �ߡ����Ԫ�ص���Ⱦ
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

	// ���붥������
	void setInfo(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, int r, int g, int b, int size);
	// �����Ƿ������Ⱦ
	void setVisible(bool visible) { this->visible_ = visible; }
	// ��״���ƺ���
	void renderShape();

private:
	void init();

private:
	RenderWidget* glWidget_ = nullptr;
	Camera* camera_ = nullptr;
	Shader* shapeVBOShader_ = nullptr;		// ������ɫ��
	GLBuffer* shapeBuffer_ = nullptr;		// ��״����

	GLenum renderType_;						// ��¼��Ⱦ����
	bool visible_;
	Eigen::MatrixXf data_;					// ��¼����������ɫ��Ϣ
	int pointNums_;							// ��¼��������
	int shapeSize_;							// ��¼����ߴ�

};

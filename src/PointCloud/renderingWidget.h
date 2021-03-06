/*
 * @Descripttion: 
 * @version: 
 * @Author: JinYiGao
 * @Date: 2021-05-29 18:36:32
 * @LastEditors: JinYiGao
 * @LastEditTime: 2021-07-31 23:34:30
 */
#pragma once

#include <iostream>
#include <memory>

#include <qevent.h>
#include <QOpenGLWidget>
#include <QOpenGlFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QPainter>
#include <QPoint>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Base/utils.h>
#include <Base/shader.h>
#include <Base/texture.h>
#include <Base/framebuffer.h>
#include <Base/glbuffer.h>
#include <Base/camera.h>
#include <Base/glutils.h>
#include <Base/tool.h>
#include <Base/tool_camera.h>
#include <Base/tool_drawPolygon.h>
#include <Base/tool_drawPolyline.h>
#include <Base/tool_delete_pcd.h>
#include <Base/tool_add_pcd.h>
#include <Base/tool_pick_pcd.h>
#include <Base/tool_edit_label.h>
#include <Base/pcdManager.h>
#include <PointCloud/pointclouds.h>
#include <PointCloud/uploader.h>
#include <PointCloud/progressiveRender.h>
#include "PointCloud/shapeRender.h"

class DBRoot;
class ToolDeletePcd;
class ToolAddPcd;

// 视口大小
struct Window {
	int width;
	int height;
};

//继承自QOpenGLFunctions可以避免每次调用opengl函数时使用前缀
class RenderWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core{
	Q_OBJECT
public:
	RenderWidget(QWidget *parent);
	RenderWidget(QWidget *parent, std::shared_ptr<PointCloud> &pcd);
	~RenderWidget();

	Window *window = new Window();
	int fps = 0;

	Camera *camera = nullptr; // 相机

	// ------------------------------------- Tool ----------------------------------------
	PcdManager *pcdManager = nullptr;
	ToolManager *toolManager = nullptr; // 工具管理

	ToolCamera *toolCamera = nullptr; // 三维相机工具
	ToolDrawPolygon *toolDrawPolygon = nullptr; // 多边形绘制工具
	ToolDrawPolyline *toolDrawPolyline = nullptr; // 多段线绘制工具
	ToolDeletePcd *toolDeletePcd = nullptr; // 点云框选删除工具
	ToolAddPcd *toolAddPcd = nullptr; // 点云追加工具
	ToolPick *toolPick = nullptr; // 鼠标点选工具
	ToolEditLabel *toolEditLabel = nullptr; // 编辑点云分类工具

	std::map<int, Tool*> extraTools; // 额外工具管理
	// -----------------------------------------------------------------------------------

// 交互信号
Q_SIGNALS:
	void pickPointCallback(Eigen::Vector3f& point);
	void drawPolygonCallback(vector<Point>& polygon);

// OpenGL绘制相关资源 
public:
	FrameBuffer *fbo = nullptr; // 离屏绘制到的帧缓冲
	FrameBuffer *fboEDL = nullptr; // EDL处理后帧缓冲
	GLBuffer *quadBuffer = nullptr; // 离屏渲染后纹理贴图quad缓冲管理

	Texture *gradientImage = nullptr; // 颜色梯度纹理, 用于着色器内获取颜色

	Shader *reprojectShader = nullptr; // 重投影着色器
	Shader *fillShader = nullptr; // 空洞填充着色器
	Shader *createVBOShader = nullptr; // 创建重投影VBO着色器

	Shader* shapeVBOShader = nullptr; // 线面着色器

	Shader *edlShader = nullptr; // EDL处理着色器
	Shader *edlShaderMSAA = nullptr; //EDL处理多重采样抗锯齿着色器

	Shader *SegmentShader = nullptr; // 裁剪着色器
	Shader *resumeSegmentShader = nullptr; // 恢复裁剪着色器

	Shader *selectShader = nullptr; // 区域选择着色器

	vector<UniformBlock*> uniformBlocks; // uniform块管理

public:
	bool addPointCloud(std::shared_ptr<PointCloud> &pcd); // 添加显示点云
	bool removePointCloud(QString name);
	void removeAllPointClouds();

	bool addPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, QString pointsName, int r = 255, int g = 128, int b = 51, int pointSize = 6);

	bool addLines(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, QString lineName, int r = 255, int g = 128, int b = 51, int pointSize = 6);
	bool removeShape(QString shapeName);
	void removeAllShapes();

	void setMSAAsamples(int samples);
	void setEDL(bool enable);
	bool getEDLState() const { return EDL_ENABLE; }

	//vector<ProgressiveRender*> getRenderList();
	std::map<QString, ProgressiveRender*> getProgressiveRenderMap();
	std::map<QString, ShapeRender*> getShapeRenderMap();

	// 获取色带
	Eigen::MatrixXf getCoolRenderColor();
	Eigen::MatrixXf getGrayRenderColor();
	Eigen::MatrixXf getHotRenderColor();
	Eigen::MatrixXf getJetRenderColor();

	int RegisterExtraTool(Tool* tool); // 额外工具注册 
	bool RemoveExtraTool(Tool* tool); // 注销额外的工具
	bool RemoveExtraToolById(int id); // 注销额外的工具 根据id

	// ------------------------------------------------------------------------------------------
	void startSegment(vector<QString> name, int mode = 0); // 对指定name点云进行裁剪
	void resumeSegment(vector<QString> name); // 恢复裁剪过的点云
	std::shared_ptr<PointCloud> createPcd(vector<QString> name);

	void resizeGL(int w, int h);

protected:
	//std::vector<ProgressiveRender*> progressiveRenderList; // 渐进渲染集合（点云、多边形mesh）
	//std::vector<ShapeRender*> shapeRenderList;			  // 直线、文字等小规模渲染集合
	std::map<QString, ProgressiveRender*> progressiveRenderMap_;
	std::map<QString, ShapeRender*> shapeRenderMap_;

	int MSAA_SAMPLES = 1; // 多重采样抗锯齿
	bool EDL_ENABLE = true; // 是否开启EDL

	bool EnableDrawPolyline = false; // 同时启用多段线绘制

	double camera_diagonal = -1;

protected:
	void initTools();

    void initializeGL();
    //void resizeGL(int w, int h);
    void paintGL();

	// 鼠标操作
	void mousePressEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseDoubleClickEvent(QMouseEvent *e);
	void wheelEvent(QWheelEvent *e);
	// 键盘事件
	void keyPressEvent(QKeyEvent *e);

private:
	Eigen::MatrixXf coolColors;
	Eigen::MatrixXf grayColors;
	Eigen::MatrixXf hotColors;
	Eigen::MatrixXf jetColors;

	Eigen::MatrixXf coolRenderColor();
	Eigen::MatrixXf grayRenderColor();
	Eigen::MatrixXf hotRenderColor();
	Eigen::MatrixXf jetRenderColor();

};
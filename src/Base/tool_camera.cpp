/*
 * @Descripttion:
 * @version:
 * @Author: JinYiGao
 * @Date: 2021-07-18 20:45:28
 * @LastEditors: JinYiGao
 * @LastEditTime: 2021-07-18 20:45:28
 */
#include "tool_camera.h"
#include <PointCloud/renderingWidget.h>
#include "Tools.h"

ToolCamera::ToolCamera(RenderWidget* glWidget) {
	this->glWidget = glWidget;
	this->camera = glWidget->camera;
}

ToolCamera::ToolCamera() {

}

ToolCamera::~ToolCamera() {

}

void ToolCamera::setCameraType(int cameratype) {
	this->cameraType = cameratype;
	//this->camera->position = Eigen::Vector3f(0, 0, 1.0);
	this->camera->arcball.setState(Quaternionf::Identity());
}

void ToolCamera::setProjection(int projMethod) {
	if (projMethod == Perspective) {
		this->camera->is_ortho = false;
	}
	else if (projMethod == Ortho) {
		this->camera->is_ortho = true;
	}
}

void ToolCamera::suspend() {
	this->isSuspend = true;
}

void ToolCamera::resume() {
	this->isSuspend = false;
}

int ToolCamera::getToolType() {
	return this->toolType;
}

void ToolCamera::mousePress(QMouseEvent* e) {
	if (isSuspend) {
		return;
	}

	//左键按下
	if (e->buttons() == Qt::LeftButton && this->cameraType != Camera2D)
	{
		camera->is_rotate = true;
		Vector2i screen(e->localPos().x(), e->localPos().y());
		camera->start_rotate(screen); //开始旋转
	}
	//右键按下
	if (e->buttons() == Qt::RightButton)
	{
		camera->is_translate = true;
		Vector2f screen(e->localPos().x(), e->localPos().y());
		camera->start_translate(screen); //开始平移
	}
	// 中键按下
	if (e->button() == Qt::MiddleButton)
	{
		//获取鼠标点击位置 
		QPoint cursorPos = QCursor::pos();
		//将鼠标全局坐标转为相对于控件的坐标 
		QPoint posToScreen = glWidget->mapFromGlobal(cursorPos);

		int onModel;
		Eigen::Vector2f screen;
		screen.x() = static_cast<float>(posToScreen.x());
		screen.y() = static_cast<float>(posToScreen.y());
		Eigen::Vector3f world = this->convert_2dTo3d(screen, onModel);

		if (onModel == 1)
		{
			camera->setRotateCenter(world);
		}
		//std::cout << world << std::endl;
	}
	glWidget->update();
}

void ToolCamera::mouseMove(QMouseEvent* e) {
	if (isSuspend) {
		return;
	}
	if (!camera->is_rotate && !camera->is_translate) {
		return;
	}
	// 左键按下移动
	if (e->type() == QEvent::MouseMove && (e->buttons() == Qt::LeftButton))
	{
		Eigen::Vector2i screen(e->localPos().x(), e->localPos().y());
		camera->motion_rotate(screen);
	}
	// 右键按下移动
	if (e->type() == QEvent::MouseMove && (e->buttons() == Qt::RightButton))
	{
		Eigen::Vector2f screen(e->localPos().x(), e->localPos().y());
		camera->motion_translate(screen);
	}

	glWidget->update();
}

void ToolCamera::mouseRelease(QMouseEvent* e) {
	if (isSuspend) {
		return;
	}
	Eigen::Vector2i screen(e->localPos().x(), e->localPos().y());
	camera->end_rotate(screen);    //结束旋转
	camera->end_translate(screen); //结束平移

	glWidget->update();
}

void ToolCamera::mouseDoubleClick(QMouseEvent* e) {
	if (isSuspend) {
		return;
	}

	if (e->button() == Qt::LeftButton) {
		Eigen::Vector2f screen2D_(e->localPos().x(), e->localPos().y());
		Eigen::Vector3f screen3D_ = camera->convert_2dTo3d(screen2D_);

		//camera->setPreTransform(getModelMatrixToOrigin(screen3D_));
		//camera->model_translation = Eigen::Vector3f::Zero();
		//Eigen::Vector3f target_ = camera->target;
		//Eigen::Vector3f up_ = camera->up;

		//camera->position = screen3D_;
	}
	glWidget->update();
}

void ToolCamera::wheelEvent(QWheelEvent* e) {
	if (isSuspend) {
		return;
	}
	// 缩放
	camera->zoom = camera->zoom * (e->angleDelta().y() > 0 ? 1.1 : 0.9);
	//通过改变相机视野 来缩放 效果好
	//camera->view_angle = std::max(0.001, camera->view_angle * (e->angleDelta().y() < 0 ? 1.1 : 0.9));
	//camera->view_angle = std::min(1000.0f, camera->view_angle);
}

// 屏幕坐标转opengl 3d坐标, onModel为1则在物体上，为-1则在屏幕空白处
Eigen::Vector3f ToolCamera::convert_2dTo3d(const Eigen::Vector2f& screen_point, int& onModel) {

	GLint xwin = static_cast<int>(screen_point.x());
	GLint ywin = camera->size.y() - static_cast<int>(screen_point.y());

	FrameBuffer* fbo = this->glWidget->fbo;
	std::vector<float> depth_temp(fbo->width * fbo->height);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glBindFramebuffer(GL_READ_FRAMEBUFFER, fbo->fbo);
	glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
	glReadPixels(0, 0, fbo->width, fbo->height, GL_DEPTH_COMPONENT, GL_FLOAT, depth_temp.data());
	glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

	//cv::Mat image_depth(height_, width_, CV_32F, depth_temp.data());
	int index = ywin * fbo->width + xwin;
	float zdepth = depth_temp[index];
	if (zdepth == 1.0)
		onModel = -1;
	else
		onModel = 1;
	//glReadPixels(xwin, ywin, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &zdepth);

	// 标准化坐标
	Vector4f screen2d;
	screen2d.x() = screen_point.x() / (float)camera->size.x() * 2 - 1.0;
	screen2d.y() = -screen_point.y() / (float)camera->size.y() * 2 + 1.0;

	Eigen::Matrix4f transform = camera->getTransform();
	screen2d.z() = zdepth * 2.0 - 1.0;
	screen2d.w() = 1.0;
	auto inverse = transform.inverse();
	Eigen::Vector4f screen3d = inverse * screen2d;
	screen3d.z() *= screen3d.w();

	return screen3d.head(3);
}

// opengl 3d坐标转屏幕坐标
Eigen::Vector2f ToolCamera::convert_3dTo2d(const Eigen::Vector3f& world_point) {
	Eigen::Vector4f pte(world_point.x(), world_point.y(), world_point.z(), 1);

	std::tuple<Eigen::Matrix4f, Eigen::Matrix4f, Eigen::Matrix4f> mvp = camera->get_mvp();
	Eigen::Matrix4f model = std::get<0>(mvp);// this->createModel(Eigen::Matrix4f::Identity(), Eigen::Vector3f::Identity(), this->zoom);
	Eigen::Matrix4f view = std::get<1>(mvp);
	Eigen::Matrix4f proj = std::get<2>(mvp);

	Eigen::Vector4f window_cord;
	window_cord = proj * view * model * pte;
	window_cord = window_cord / window_cord(3);
	window_cord[0] = (window_cord[0] + 1.0) / 2.0 * (float)camera->size.x();
	window_cord[1] = -(window_cord[1] - 1.0) / 2.0 * (float)camera->size.y();
	window_cord[2] = (window_cord[2] + 1.0) / 2.0;

	return window_cord.head(2);
}
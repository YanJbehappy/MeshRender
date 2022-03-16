/*
 * @Descripttion: 点云渲染窗体部件
 * @version: 
 * @Author: JinYiGao
 * @Date: 2021-05-29 18:36:41
 * @LastEditors: JinYiGao
 * @LastEditTime: 2021-07-31 23:36:30
 */

#include "progressiveRender.h"
#include "renderingWidget.h"
//#include <Main/DBRoot.h>
//#include <Main/mainwindow.h>

//继承自QOpenGLFunctions可以避免每次调用opengl函数时使用前缀
RenderWidget::RenderWidget(QWidget *parent) : QOpenGLWidget(parent){
	// 初始化相机
	this->camera = new Camera();
	this->toolManager = nullptr;
}

RenderWidget::RenderWidget(QWidget *parent, std::shared_ptr<PointCloud> &pcd) : QOpenGLWidget(parent){
	//this->pcd = pcd;
	addPointCloud(pcd);
	// 初始化相机
	this->camera = new Camera();
}

// 析构函数
RenderWidget::~RenderWidget() {
	if (toolManager) {
		delete toolManager;
	}
}

// 初始化工具
void RenderWidget::initTools() {
	// 设置焦点接收
	this->setFocusPolicy(Qt::StrongFocus);

	pcdManager = PcdManager::GetInstance();

	toolManager = new ToolManager(this);

	toolCamera = new ToolCamera(this);
	toolDrawPolygon = new ToolDrawPolygon();
	toolDrawPolyline = new ToolDrawPolyline();
	toolDeletePcd = new ToolDeletePcd(this);
	toolAddPcd = new ToolAddPcd(this);
	toolPick = new ToolPick(this);
	toolEditLabel = new ToolEditLabel(this);

	// 工具注册
	toolManager->register_tool(CameraTool, toolCamera);
	toolManager->register_tool(DrawPolygonTool, toolDrawPolygon);
	toolManager->register_tool(DrawPolylineTool, toolDrawPolyline);
	toolManager->register_tool(DeletePcdTool, toolDeletePcd);
	toolManager->register_tool(AddPcdTool, toolAddPcd);
	toolManager->register_tool(PickPointTool, toolPick);
	toolManager->register_tool(EditLabelTool, toolEditLabel);

	// 生成渲染色带
	coolColors = coolRenderColor();
	grayColors = grayRenderColor();
	hotColors = hotRenderColor();
	jetColors = jetRenderColor();

	connect(toolPick, &ToolPick::pickPointCallback, this, [&](Eigen::Vector3f point) { emit pickPointCallback(point); }, Qt::UniqueConnection);
	connect(toolDrawPolygon, &ToolDrawPolygon::drawPolygonCallback, this, [&](vector<Point>& polygon) { emit drawPolygonCallback(polygon); }, Qt::UniqueConnection);
	// 默认工具切换为相机
	toolManager->changeTool(CameraTool);
}

// 添加点云
bool RenderWidget::addPointCloud(std::shared_ptr<PointCloud> &pcd) {
	ProgressiveRender *progressiveRender = new ProgressiveRender(this, pcd);
	// 防止同时写入读取冲突
	auto copyrenderList = progressiveRenderMap_;
	std::map<QString, ProgressiveRender*>::iterator cloud_it = copyrenderList.find(pcd->name);

	if (cloud_it != copyrenderList.end())
	{
		qDebug() << tr("[addPointCloud] A cloud with id ") + pcd->name + tr(" already exists!") << endl;
		return false;
	}

	copyrenderList.insert(std::make_pair(pcd->name, progressiveRender));
	progressiveRenderMap_ = copyrenderList;
	//renderList.emplace_back(progressiveRender);

	if (progressiveRenderMap_.size() == 1) {
		camera->setPreTransform(pcd->getModelMatrixToOrigin()); // 以第一个点云为基准 平移至中心
	}

	//// 初始化相机设置
	double diagonal = pcd->boundingBox.diagonal().norm();
	if (diagonal > camera_diagonal)
	{
		camera->zoom = 1.0 / diagonal;
		camera->scene_bbox = pcd->boundingBox;
		camera_diagonal = diagonal;
	}

}

bool RenderWidget::removePointCloud(QString name) {
	std::map<QString, ProgressiveRender*>::iterator cloud_it = progressiveRenderMap_.find(name);

	if (cloud_it == progressiveRenderMap_.end())
	{
		//qDebug() << tr("[removePointCloud] A cloud with id ") + name + tr(" doesn't exist!") << endl;
		return false;
	}

	delete cloud_it->second;
	cloud_it->second = nullptr;
	cloud_it = progressiveRenderMap_.erase(cloud_it);    //删除元素，返回值指向已删除元素的下一个位置

	return true;
}

void RenderWidget::removeAllPointClouds()
{
	auto cloud_it = progressiveRenderMap_.begin();
	while (cloud_it != progressiveRenderMap_.end()) {
		delete cloud_it->second;
		cloud_it->second = nullptr;
		progressiveRenderMap_.erase(cloud_it++);    //删除元素，返回值指向已删除元素的下一个位置
	}

	//vector<ProgressiveRender*>::iterator it = progressiveRenderList.begin();
	//for (int i = 0; i < progressiveRenderList.size(), it != progressiveRenderList.end(); i++) {
	//	delete progressiveRenderList[i];
	//	progressiveRenderList[i] = nullptr;
	//	it = progressiveRenderList.erase(it);    //删除元素，返回值指向已删除元素的下一个位置
	//	i--;
	//}
}

bool RenderWidget::addPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, QString pointsName, int r, int g, int b, int pointSize)
{
	// 初始化渲染对象
	ShapeRender* shapeRender = new ShapeRender(this, GL_POINTS);
	shapeRender->setInfo(points, r, g, b, pointSize);

	// 防止同时写入读取冲突
	auto copyShapeRenderMap = shapeRenderMap_;
	std::map<QString, ShapeRender*>::iterator shape_it = copyShapeRenderMap.find(pointsName);
	if (shape_it != copyShapeRenderMap.end())
	{
		qDebug() << tr("[addPoints] A shape with id ") + pointsName + tr(" already exists!") << endl;
		return false;
	}
	copyShapeRenderMap.insert(std::make_pair(pointsName, shapeRender));
	shapeRenderMap_ = copyShapeRenderMap;
}

bool RenderWidget::addLines(pcl::PointCloud<pcl::PointXYZ>::Ptr& points, QString lineName, int r, int g, int b, int pointSize)
{
	// 初始化渲染对象
	ShapeRender* shapeRender = new ShapeRender(this, GL_LINES);
	shapeRender->setInfo(points, r, g, b, pointSize);

	// 防止同时写入读取冲突
	auto copyShapeRenderMap = shapeRenderMap_;
	std::map<QString, ShapeRender*>::iterator shape_it = copyShapeRenderMap.find(lineName);
	if (shape_it != copyShapeRenderMap.end())
	{
		qDebug() << tr("[addLines] A shape with id ") + lineName + tr(" already exists!") << endl;
		return false;
	}
	copyShapeRenderMap.insert(std::make_pair(lineName, shapeRender));
	shapeRenderMap_ = copyShapeRenderMap;
}

bool RenderWidget::removeShape(QString shapeName)
{
	std::map<QString, ShapeRender*>::iterator shape_it = shapeRenderMap_.find(shapeName);

	if (shape_it == shapeRenderMap_.end())
	{
		//qDebug() << tr("[removeLine] A shape with id ") + lineName + tr(" doesn't exist!") << endl;
		return false;
	}

	delete shape_it->second;
	shape_it->second = nullptr;
	shape_it = shapeRenderMap_.erase(shape_it);    //删除元素，返回值指向已删除元素的下一个位置

	return true;
}

void RenderWidget::removeAllShapes()
{
	auto cloud_it = shapeRenderMap_.begin();
	while (cloud_it != shapeRenderMap_.end()) {
		delete cloud_it->second;
		cloud_it->second = nullptr;
		shapeRenderMap_.erase(cloud_it++);    //删除元素，返回值指向已删除元素的下一个位置
	}
}

void RenderWidget::initializeGL()
{
	// 初始化
	initializeOpenGLFunctions();
	// 初始化工具
	initTools();
	// -----------------------------离屏渲染纹理贴图顶点信息缓冲区设置---------------------------------
	// quadVertices
	float vertices[] = {
		//顶点  //纹理
		-1,-1,0,  0,0,
		1, -1,0,  1,0,
		1,1,0,    1,1,

		-1,-1,0,  0,0,
		1,1,0,    1,1,
		-1,1,0,   0,1
	};
	quadBuffer = new GLBuffer();
	vector<GLBufferAttribute> quadAttributes;
	GLBufferAttribute quadAttribute1("xyz", 0, 3, GL_FLOAT, GL_FALSE, 20, 0);
	quadAttributes.emplace_back(quadAttribute1);
	GLBufferAttribute quadAttributu2("uv", 1, 2, GL_FLOAT, GL_FALSE, 20, 12);
	quadAttributes.emplace_back(quadAttributu2);
	// 传入顶点并设置属性解析
	quadBuffer->set(&vertices, sizeof(vertices), quadAttributes, 6);
	// -------------------------------------------------------------------------------------------------

	// ------------------------------------- 离屏渲染帧缓冲 --------------------------------------------
	fbo = new FrameBuffer();
	fbo->setNumColorAttachments(2); // 一张用于输出纹理图像 一张用于记录索引数据

	fboEDL = new FrameBuffer(); // 绘制EDL处理结果的帧缓冲
	// -------------------------------------------------------------------------------------------------

	// --------------------------------------- 着色器创建 ----------------------------------------------
	reprojectShader = new Shader(":/PointCloud/shaders/reproject.vs", ":/PointCloud/shaders/reproject.fs");
	fillShader = new Shader(":/PointCloud/shaders/fill.vs", ":/PointCloud/shaders/fill.fs");
	createVBOShader = new Shader(":/PointCloud/shaders/create_vbo.cs");
	shapeVBOShader = new Shader(":/PointCloud/shaders/shape_vbo.vs", ":/PointCloud/shaders/shape_vbo.fs");
	edlShader = new Shader(":/PointCloud/shaders/edl.vs", ":/PointCloud/shaders/edl.fs");
	edlShaderMSAA = new Shader(":/PointCloud/shaders/edl.vs", ":/PointCloud/shaders/edlMSAA.fs");
	SegmentShader = new Shader(":/PointCloud/shaders/segment.cs");
	resumeSegmentShader = new Shader(":/PointCloud/shaders/resume_segment.cs");
	selectShader = new Shader(":/PointCloud/shaders/select.cs");
	// -------------------------------------------------------------------------------------------------

	// ---------------------------------------- 纹理创建 -----------------------------------------------
	gradientImage = new Texture(":/PointCloud/images/gradient_spectral_2d.png");
	gradientImage->setTexParameteri(GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	gradientImage->setTexParameteri(GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	// -------------------------------------------------------------------------------------------------

	//缓存清除
	glClearColor(60 / 255.0, 63 / 255.0, 65 / 255.0, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void RenderWidget::resizeGL(int w, int h)
{
	if (!toolDrawPolygon)
		return;

	if (toolDrawPolygon->polygon.size() > 0) {
		for (int i = 0; i < toolDrawPolygon->polygon.size(); i++) {
			toolDrawPolygon->polygon[i].x *= (w / (float)window->width);
			toolDrawPolygon->polygon[i].y *= (h / (float)window->height);
		}
	}

	window->width = w;
	window->height = h;

    //用于更新投影矩阵或者其他相关大小设置
    camera->size = {w, h};
    camera->arcball.setSize(Vector2i(w, h));

	// 更新fbo大小
	fbo->setSize(w, h);
	fboEDL->setSize(w, h); // fboEDL大小需要跟fbo一致
}

void RenderWidget::paintGL()
{
	// 无点云 返回
	if (progressiveRenderMap_.size() == 0 && shapeRenderMap_.size() == 0) {
		return;
	}

	QPainter painter;
	painter.begin(this);
	painter.beginNativePainting();

	// 开启着色器点大小获取
	glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
	// 开启深度检测
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	// 设置帧缓冲样本数 --- 采样方式
	fbo->setSamples(MSAA_SAMPLES);
	// 绑定当前渲染到帧缓冲 ------------------- 离屏渲染
	fbo->bind();
	//缓存清除
	glClearColor(60 / 255.0, 63 / 255.0, 65 / 255.0, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//for (int i = 0; i < progressiveRenderList.size(); i++) {
	//	progressiveRenderList[i]->renderPointCloudProgressive();
	//}
	for (auto cloud_it : progressiveRenderMap_) {
		cloud_it.second->renderPointCloudProgressive();
	}

	for (auto shape_it : shapeRenderMap_) {
		shape_it.second->renderShape();
	}

	// ToolManager gl Draw
	toolManager->gl_draw();

	// 开启 EDL
	if (EDL_ENABLE) {
		// 根据fbo样本数选择采用哪个shader
		Shader *shader = fbo->samples == 1 ? edlShader : edlShaderMSAA;
		// 切换当前渲染帧缓冲为EDL的fbo
		glBindFramebuffer(GL_FRAMEBUFFER, fboEDL->fbo);
		// 切换EDL处理着色器
		shader->bind();
		GLenum TextureType = fbo->samples == 1 ? GL_TEXTURE_2D : GL_TEXTURE_2D_MULTISAMPLE;
		// 纹理传入着色器
		shader->setUniformValue("uColor", 0); // 颜色纹理
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(TextureType, fbo->textures[0]);
		shader->setUniformValue("uDepth", 1); // 深度图
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(TextureType, fbo->depth);
		
		glDisable(GL_DEPTH_TEST); // 禁止深度测试
		glDepthMask(false); // 禁止深度写入
		glDisable(GL_CULL_FACE); // 禁止面剔除

		// 写入uniform块数据
		float edlStrength = 0.4;
		shader->setUniformBlockValue(uniformBlocks, "shader_data", "shader_data.edlStrength",&edlStrength);
		float samples = fbo->samples;
		shader->setUniformBlockValue(uniformBlocks, "shader_data", "shader_data.msaaSampleCount", &samples);
		// 绘制
		glBindVertexArray(quadBuffer->VAO);
		glDrawArrays(GL_TRIANGLES, 0, quadBuffer->vertexCount);
		glBindVertexArray(0);

		glEnable(GL_DEPTH_TEST);
		glDepthMask(true);

		// 复制fbo帧缓冲内容到屏幕帧缓冲
		glBlitNamedFramebuffer(fboEDL->fbo, defaultFramebufferObject(),
			0, 0, fboEDL->width, fboEDL->height,
			0, 0, window->width, window->height,
			GL_COLOR_BUFFER_BIT, GL_LINEAR);
	}
	else {
		// 复制fbo帧缓冲内容到屏幕帧缓冲
		glBlitNamedFramebuffer(fbo->fbo, defaultFramebufferObject(),
			0, 0, fbo->width, fbo->height,
			0, 0, window->width, window->height,
			GL_COLOR_BUFFER_BIT, GL_LINEAR);
	}

	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);

	painter.endNativePainting();

	// toolManager绘制二维图形 (ps: 一定记得QPainter绘制时解绑opengl相关变量)
	toolManager->draw(&painter);
	for (auto &pair : this->extraTools) {
		pair.second->draw(&painter);
	}
	painter.end();
	
	update();
}

// 相关设置
void RenderWidget::setMSAAsamples(int samples) {
	this->MSAA_SAMPLES = samples;
}

void RenderWidget::setEDL(bool enable) {
	this->EDL_ENABLE = enable;
}

int RenderWidget::RegisterExtraTool(Tool* tool) {
	int id = this->extraTools.size();
	tool->activate();
	this->extraTools.emplace(id, tool);
	return id;
}

bool RenderWidget::RemoveExtraTool(Tool *tool) {
	for (auto pair = this->extraTools.begin(); pair != this->extraTools.end();) {
		if (pair->second == tool) {
			pair->second->deactivate();
			delete pair->second;
			pair->second = nullptr;
			this->extraTools.erase(pair++);

			return true;
		}
		else {
			pair++;
		}
	}
	return false;
}

bool RenderWidget::RemoveExtraToolById(int id) {
	for (auto pair = this->extraTools.begin(); pair != this->extraTools.end();) {
		if (pair->first == id) {
			pair->second->deactivate();
			delete pair->second;
			pair->second = nullptr;
			this->extraTools.erase(pair++);

			return true;
		}
		else {
			pair++;
		}
	}
	return false;
}

// 鼠标交互控制
void RenderWidget::mousePressEvent(QMouseEvent *e) {
	toolManager->mousePress(e);

	for (auto &pair : this->extraTools) {
		pair.second->mousePress(e);
	}
}

void RenderWidget::mouseReleaseEvent(QMouseEvent *e)
{
	toolManager->mouseRelease(e);

	for (auto &pair : this->extraTools) {
		pair.second->mouseRelease(e);
	}
}

void RenderWidget::mouseMoveEvent(QMouseEvent *e)
{
	toolManager->mouseMove(e);

	for (auto &pair : this->extraTools) {
		pair.second->mouseMove(e);
	}
}

void RenderWidget::mouseDoubleClickEvent(QMouseEvent *e) {
	toolManager->mouseDoubleClick(e);

	for (auto &pair : this->extraTools) {
		pair.second->mouseDoubleClick(e);
	}
}

void RenderWidget::wheelEvent(QWheelEvent *e)
{
	toolManager->wheelEvent(e);

	for (auto &pair : this->extraTools) {
		pair.second->wheelEvent(e);
	}
}

void RenderWidget::keyPressEvent(QKeyEvent *e) {
	this->toolManager->keyPress(e);

	/*if (toolManager->getToolType() != DrawPolylineTool && EnableDrawPolyline) {
		toolDrawPolyline->keyPress(e);
	}*/
	for (auto &pair : this->extraTools) {
		pair.second->keyPress(e);
	}
}

// 调用progressive函数进行裁剪
void RenderWidget::startSegment(vector<QString> name, int mode) {
	if (toolManager->getToolType() != DrawPolygonTool) {
		return;
	}
	for (int i = 0; i < name.size(); i++) {
		std::map<QString, ProgressiveRender*>::iterator cloud_it = progressiveRenderMap_.find(name[i]);
		if (cloud_it == progressiveRenderMap_.end())
		{
			qDebug() << tr("[startSegment] A cloud with name ") + name[i] + tr(" doesn't exists") << endl;
			continue;
		}
		cloud_it->second->Segment(toolDrawPolygon->polygon, mode);
		//for (int j = 0; j < progressiveRenderList.size(); j++) {
		//	if (name[i] == progressiveRenderList[j]->name) {
		//		progressiveRenderList[j]->Segment(toolDrawPolygon->polygon, mode);
		//	}
		//}
	}
}

void RenderWidget::resumeSegment(vector<QString> name) {
	for (int i = 0; i < name.size(); i++) {
		std::map<QString, ProgressiveRender*>::iterator cloud_it = progressiveRenderMap_.find(name[i]);
		if (cloud_it == progressiveRenderMap_.end())
		{
			qDebug() << tr("[resumeSegment] A cloud with name ") + name[i] + tr(" doesn't exists") << endl;
			continue;
		}
		cloud_it->second->resumeSegment();
		//for (int j = 0; j < progressiveRenderList.size(); j++) {
		//	if (name[i] == progressiveRenderList[j]->name) {
		//		progressiveRenderList[j]->resumeSegment();
		//	}
		//}
	}
}

std::shared_ptr<PointCloud> RenderWidget::createPcd(vector<QString> name) {
	vector<std::shared_ptr<PointCloud>>pcds;
	for (int i = 0; i < name.size(); i++) {
		std::map<QString, ProgressiveRender*>::iterator cloud_it = progressiveRenderMap_.find(name[i]);
		if (cloud_it == progressiveRenderMap_.end())
		{
			qDebug() << tr("[createPcd] A cloud with name ") + name[i] + tr(" doesn't exists") << endl;
			continue;
		}
		std::shared_ptr<PointCloud> pcd_ = cloud_it->second->createPcdFromBuffer();
		if (pcd_ != nullptr) {
			pcds.emplace_back(pcd_);
		}
		//for (int j = 0; j < progressiveRenderList.size(); j++) {
		//	if (name[i] == progressiveRenderList[j]->name) {
		//		std::shared_ptr<PointCloud> pcd_ = progressiveRenderList[j]->createPcdFromBuffer();
		//		if (pcd_ != nullptr) {
		//			pcds.emplace_back(pcd_);
		//		}
		//	}
		//}
	}
	// merge pcds
	std::shared_ptr<PointCloud> pcd = pcdManager->merge_poindcloud(pcds);

	return pcd;
}

//vector<ProgressiveRender*> RenderWidget::getRenderList() {
//	return this->progressiveRenderList;
//}

//---------------------------private--------------------------------//
Eigen::MatrixXf RenderWidget::coolRenderColor()
{
	Eigen::MatrixXf colors;
	colors.resize(4, 256);
	float buff = 256.0 / 255.0;
	for (int i = 0; i < 256; i++)
	{
		colors.col(i) = Eigen::Vector4f(i * buff, 255 - i * buff, 255, 255) / 255.0;
	}
	return colors;
}

Eigen::MatrixXf RenderWidget::grayRenderColor()
{
	Eigen::MatrixXf colors;
	colors.resize(4, 256);
	float buff = 190.0 / (255.0);
	for (int i = 0; i < 256; i++)
	{
		colors.col(i) = Eigen::Vector4f(i * buff, i * buff, i * buff, 255) / 255.0;
	}
	return colors;
}

Eigen::MatrixXf RenderWidget::hotRenderColor()
{
	Eigen::MatrixXf colors;
	colors.resize(4, 256);
	float buf = 3;
	for (int i = 0; i < 86; i++) //86
	{
		colors.col(i) = Eigen::Vector4f(i * 3, 0, 0, 255) / 255.0;
	}
	int it = 0;
	for (int i = 87; i < 173; i++) //86
	{
		colors.col(i) = Eigen::Vector4f(255, it * 3, 0, 255) / 255.0;
		it = it + 1;
	}
	it = 0;
	for (int i = 173; i < 256; i++)
	{
		colors.col(i) = Eigen::Vector4f(255, 255, it * 3, 255) / 255.0;
		it = it + 1;
	}
	return colors;
}

Eigen::MatrixXf RenderWidget::jetRenderColor()
{
	Eigen::MatrixXf colors;
	colors.resize(4, 256);
	int it = 0;
	for (int i = 0; i < 32; i++)
	{
		colors.col(i) = Eigen::Vector4f(0, 0, 128 + 4 * it, 255) / 255.0;
		it = it + 1;
	}

	colors.col(32) = Eigen::Vector4f(0, 0, 255, 255) / 255.0;

	it = 0;
	for (int i = 33; i < 96; i++)
	{
		colors.col(i) = Eigen::Vector4f(0, 4 + 4 * it, 255, 255) / 255.0;
		it = it + 1;
	}

	colors.col(96) = Eigen::Vector4f(2, 255, 254, 255) / 255.0;

	it = 0;
	for (int i = 97; i < 159; i++)
	{
		colors.col(i) = Eigen::Vector4f(6 + 4 * it, 255, 250 - 4 * it, 255) / 255.0;
		it = it + 1;
	}

	colors.col(159) = Eigen::Vector4f(254, 255, 1, 255) / 255.0;

	it = 0;
	for (int i = 160; i < 224; i++)
	{
		colors.col(i) = Eigen::Vector4f(255, 252 - 4 * it, 0, 255) / 255.0;
		it = it + 1;
	}

	it = 0;
	for (int i = 224; i < 256; i++)
	{
		colors.col(i) = Eigen::Vector4f(252 - 4 * it, 0, 0, 255) / 255.0;
		it = it + 1;
	}
	return colors;
}

//---------------------------public--------------------------------//
std::map<QString, ProgressiveRender*> RenderWidget::getProgressiveRenderMap() {
	return this->progressiveRenderMap_;
}

std::map<QString, ShapeRender*> RenderWidget::getShapeRenderMap()
{
	return this->shapeRenderMap_;
}

Eigen::MatrixXf RenderWidget::getCoolRenderColor()
{
	return this->coolColors;
}

Eigen::MatrixXf RenderWidget::getGrayRenderColor()
{
	return this->grayColors;
}

Eigen::MatrixXf RenderWidget::getHotRenderColor()
{
	return hotColors;
}

Eigen::MatrixXf RenderWidget::getJetRenderColor()
{
	return jetColors;
}

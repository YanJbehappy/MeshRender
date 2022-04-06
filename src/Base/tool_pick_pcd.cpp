/*
 * @Descripttion: 
 * @version: 
 * @Author: JinYiGao
 * @Date: 2021-09-10 16:44:33
 * @LastEditors: JinYiGao
 * @LastEditTime: 2021-09-10 16:44:33
 */
#include <Base/tool_pick_pcd.h>
#include <PointCloud/renderingWidget.h>
#include <PointCloud/progressiveRender.h>
#include <Base/shader.h>
#include <Base/glbuffer.h>

static const char *vertShader = "#version 450 core \n"
"layout(location = 0) in vec3 aPos; \n"
"uniform mat4 transform; \n"
"void main() \n"
"{ \n"
"	gl_Position = transform * vec4(aPos, 1.0); \n"
"	gl_PointSize = 6.0; \n"
"}\n";

static const char *fragShader = "#version 450 core \n"
"out vec4 fragColor; \n"
"void main() \n"
"{ \n"
"	fragColor = vec4(0.0, 1.0, 0.0, 1.0); \n"
"} \n";

ToolPick::ToolPick(RenderWidget *glWidget) {
	initializeOpenGLFunctions();

	this->glWidget = glWidget;

	//this->drawPointShader = new Shader();
	//this->drawPointShader->compileShaderFromSourceCode(vertShader, fragShader);

	//this->glbuffer = new GLBuffer();
	//glBufferAttributes.emplace_back(GLBufferAttribute("position", 0, 3, GL_FLOAT, GL_FALSE, 3, 0));
}

ToolPick::~ToolPick() {
	//delete drawPointShader;
}

void ToolPick::setInfo(QString pcdname)
{
	this->cloudName_ = pcdname;
}

int ToolPick::getToolType() {
	return this->toolType;
}

void ToolPick::activate() {
	this->activated = true;
}

void ToolPick::deactivate() {
	this->reset();
}

void ToolPick::suspend() {
	this->activated = false;
}

void ToolPick::resume() {
	this->activated = true;
}

void ToolPick::reset() {
	this->activated = false;
	this->point = Eigen::Vector3f::Zero();
}

void ToolPick::draw(QPainter *painter) {
	int width = this->glWidget->window->width;
	int height = this->glWidget->window->height;
	painter->setPen(QPen(Qt::white, 1));
	// ״̬��ʾ
	painter->drawText(QRectF(QPointF(width / 2.0 - 100, 10), QPointF(width / 2.0 + 100, 30)), "Picking ... Press [ESC] Exit");
	// ��ѡ������ʾ
	auto offset = this->glWidget->getProgressiveRenderMap().at(cloudName_)->getCurrentPcd()->offset;
	auto point = this->point + offset;
	painter->drawText(QRectF(QPointF(width - 300, height - 40), QPointF(width -10, height - 18)), "Pick Point X:" + QString::number(point.x()) + " Y:" + QString::number(point.y()) + " Z:" + QString::number(point.z()));
	
	
}

void ToolPick::gl_draw() {
	if (!activated) {
		return;
	}
	//// ȡ����Ȳ��� ��ѡ�ĵ���Զ��ǰ��
	//glDisable(GL_DEPTH_TEST);
	//this->glbuffer->set(Eigen::MatrixXf(point), 3 * GL_FLOAT, this->glBufferAttributes, 1);
	//this->drawPointShader->bind();
	//this->drawPointShader->setUniformValue("transform", this->glWidget->camera->getTransform());
	//this->glbuffer->bind();
	//glDrawArrays(GL_POINTS, 0, this->glbuffer->vertexCount);
	//glEnable(GL_DEPTH_TEST);
}

// ����¼�
static int clickX = -1, clickY = -1;
void ToolPick::mousePress(QMouseEvent *e) {
	if (!activated) {
		return;
	}
	// �������
	if (e->buttons() == Qt::LeftButton) {
		clickX = e->localPos().x();
		clickY = e->localPos().y();
	}
	else {
		clickX = -1;
		clickY = -1;
	}

	this->glWidget->toolCamera->mousePress(e);
}

void ToolPick::mouseRelease(QMouseEvent *e) {
	this->glWidget->toolCamera->mouseRelease(e);
	
	if (clickX == e->localPos().x() && clickY == e->localPos().y()) {
		QPointF mousePos = e->localPos();
		vector<QPointF> nearPos = spiralTraversal(mousePos, 4);

		FrameBuffer* fbo = this->glWidget->fbo;
		GLubyte* indexData = new GLubyte[fbo->width * fbo->height * 4];
		glGetTextureImage(fbo->textures[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, fbo->height * fbo->width * 4, indexData);
		auto positions = glWidget->getProgressiveRenderMap().at(cloudName_)->getCurrentPcd()->position; // Get Pick Point
		auto indexTable = glWidget->getProgressiveRenderMap().at(cloudName_)->getCurrentPcd()->indexTable;
		for (int nearIndex = 0; nearIndex < nearPos.size(); nearIndex++)
		{
			QPointF nearPoint = nearPos[nearIndex];
			auto x = nearPoint.x();
			auto y = fbo->height - nearPoint.y();

			int i = 4 * fbo->width * y + 4 * x; // image index
			uint index = indexData[i] | (indexData[i + 1] << 8) | (indexData[i + 2] << 16) | (indexData[i + 3] << 24); // ���Һ�� index

			if (index < positions.cols() && index > 0) {
				this->point = positions.col(indexTable[index]);
				//std::cout << index << std::endl;
				//std::cout << this->point << std::endl;
				auto offset = this->glWidget->getProgressiveRenderMap().at(cloudName_)->getCurrentPcd()->offset;
				emit pickPointCallback(point + offset);
				break;
			}
		}

		delete indexData;
	}
}

void ToolPick::mouseMove(QMouseEvent *e) {
	this->glWidget->toolCamera->mouseMove(e);
}

void ToolPick::wheelEvent(QWheelEvent *e) {
	this->glWidget->toolCamera->wheelEvent(e);
}

// �����¼�
void ToolPick::keyPress(QKeyEvent *e) {
	if (!activated) {
		return;
	}
	if (e->key() == Qt::Key_Escape) {
		this->glWidget->toolManager->changeTool(CameraTool);
	}
}

// �����ĵ�������������ȡ��Χ�����������꣬rings��ʾ��������
std::vector<QPointF> ToolPick::spiralTraversal(QPointF& centerPos, int rings)
{
	vector<QPointF> spiralIndexSquare;
	// �Ƚ����ĵ�����б�
	spiralIndexSquare.emplace_back(centerPos);
	double x0 = centerPos.x();
	double y0 = centerPos.y();
	for (int i = 1; i < rings; i++)
	{
		// �����ϲ�
		for (int upX = static_cast<int>(x0) - i; upX < (static_cast<int>(x0) + i); upX++) {
			QPointF up(static_cast<double>(upX), y0 - static_cast<double>(i));
			spiralIndexSquare.emplace_back(up);
		}
		// �����Ҳ�
		for (int rightY = static_cast<int>(y0) - i; rightY < (static_cast<int>(y0) + i); rightY++) {
			QPointF right(x0 + static_cast<double>(i), static_cast<double>(rightY));
			spiralIndexSquare.emplace_back(right);
		}
		// �����²�
		for (int bottomX = static_cast<int>(x0) - i + 1; bottomX < (static_cast<int>(x0) + i + 1); bottomX++) {
			QPointF bottom(static_cast<double>(bottomX), y0 + static_cast<double>(i));
			spiralIndexSquare.emplace_back(bottom);
		}
		// ������
		for (int leftY = static_cast<int>(y0) - i + 1; leftY < (static_cast<int>(y0) + i + 1); leftY++) {
			QPointF left(x0 - static_cast<double>(i), static_cast<double>(leftY));
			spiralIndexSquare.emplace_back(left);
		}
	}

	return spiralIndexSquare;
}


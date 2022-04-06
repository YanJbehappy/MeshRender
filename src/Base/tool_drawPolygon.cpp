/*
 * @Descripttion:
 * @version:
 * @Author: JinYiGao
 * @Date: 2021-07-18 20:46:34
 * @LastEditors: JinYiGao
 * @LastEditTime: 2021-07-18 20:46:35
 */
#include "tool_drawPolygon.h"
#include <PointCloud/renderingWidget.h>
#include "Tools.h"

ToolDrawPolygon::ToolDrawPolygon() {
	this->polygon.clear();
}

ToolDrawPolygon::~ToolDrawPolygon() {

}

void ToolDrawPolygon::activate() {
	this->isActivate = true;
	this->drawing = true;
}

void ToolDrawPolygon::deactivate() {
	reset();
	this->isActivate = false;
}

// 重置
void ToolDrawPolygon::reset() {
	this->polygon.clear();
	this->drawing = false;
}

// 暂停绘制
void ToolDrawPolygon::suspend() {
	this->drawing = false;
}

// 恢复绘制
void ToolDrawPolygon::resume() {
	this->drawing = true;
}

void ToolDrawPolygon::setDrawType(DrawShapes type)
{
	this->drawType = type;
}

// 获取Tool类型
int ToolDrawPolygon::getToolType() {
	return type;
}

// 键鼠操作
void ToolDrawPolygon::mousePress(QMouseEvent* e) {
	if (!isActivate) {
		return;
	}
	// 双击完成绘制多边形后，在进行其他操作之前再次单击即可重绘
	if (drawing == false) {
		polygon.clear();
		this->drawing = true;
	}
	if (drawing && e->buttons() == Qt::LeftButton) {
		Point point(e->localPos().x(), e->localPos().y());
		switch (drawType)
		{
		case DrawShapes::DRAW_Circle:
		case DrawShapes::DRAW_Rectangle:
			polygon.emplace_back(point);
			break;
		case DrawShapes::DRAW_Polygon:
			polygon.emplace_back(point);
			break;
		default:
			break;
		}
	}
}

void ToolDrawPolygon::mouseRelease(QMouseEvent* e)
{
	if (!isActivate) {
		return;
	}
	if (drawing) {
		Point point(e->localPos().x(), e->localPos().y());
		switch (drawType)
		{
		case DrawShapes::DRAW_Circle:
		{
			polygon.pop_back();
			polygon.emplace_back(point);
			suspend(); // 暂停绘制

			// 画圆
			Point circle_center;
			circle_center.x = (polygon[0].x + polygon[1].x) / 2.0;
			circle_center.y = (polygon[0].y + polygon[1].y) / 2.0;
			int radius = static_cast<int>(std::abs(polygon[0].x - polygon[1].x) / 2.0);
			vector<Point> circle = Bresenham_Circle(circle_center, radius);

			emit drawPolygonCallback(circle);
			break;
		}
		case DrawShapes::DRAW_Rectangle:
		{
			polygon.pop_back();
			polygon.emplace_back(point);
			suspend(); // 暂停绘制

			// 画矩形
			vector<Point> rectangle;
			rectangle.emplace_back(polygon[0]);
			rectangle.emplace_back(polygon[1].x, polygon[0].y);
			rectangle.emplace_back(polygon[1]);
			rectangle.emplace_back(polygon[0].x, polygon[1].y);

			emit drawPolygonCallback(rectangle);
			break;
		}
		default:
			break;
		}
	}
}

void ToolDrawPolygon::mouseMove(QMouseEvent* e) {
	if (!isActivate) {
		return;
	}
	if (drawing) {
		Point point(e->localPos().x(), e->localPos().y());
		if (polygon.size() == 1) {
			polygon.emplace_back(point);
		}
		else if (polygon.size() > 1) {
			polygon.pop_back();
			polygon.emplace_back(point);
		}
	}
}

void ToolDrawPolygon::mouseDoubleClick(QMouseEvent* e) {
	if (!isActivate) {
		return;
	}
	if (drawing) {
		polygon.pop_back();
		suspend(); // 暂停绘制
		emit drawPolygonCallback(polygon);
	}

}

// 键盘事件
void ToolDrawPolygon::keyPress(QKeyEvent* e) {
	if (!isActivate) {
		return;
	}
	if (e->key() == Qt::Key_Escape) {
		this->deactivate();
	}
}

// QPainter 绘制
void ToolDrawPolygon::draw(QPainter* painter) {
	if (!isActivate) {
		return;
	}
	// Text
	auto device = painter->device();
	int width = device->width();
	int height = device->height();
	painter->setPen(QPen(Qt::white, 1));
	// 状态显示
	painter->drawText(QRectF(QPointF(width / 2.0 - 150, 10), QPointF(width / 2.0 + 150, 30)), "Drawing Polygon ... Press [ESC] Exit");

	if (polygon.size() > 0) {
		painter->setPen(QPen(Qt::green, 1));
		painter->setBrush(QBrush(QColor(0, 255, 0, 120)));

		switch (drawType)
		{
		case DrawShapes::DRAW_Circle:
		{
			QPointF center;
			center.setX((polygon[0].x + polygon[1].x) / 2.0);
			center.setY((polygon[0].y + polygon[1].y) / 2.0);
			int radius = static_cast<int>(std::abs(polygon[0].x - polygon[1].x) / 2.0);
			painter->drawEllipse(center, radius, radius);
			break;
		}
		case DrawShapes::DRAW_Rectangle:
		{
			QPointF topLeft(polygon[0].x, polygon[0].y);
			QPointF bottomRight(polygon[1].x, polygon[1].y);
			QRectF pts(topLeft, bottomRight);
			painter->drawRect(pts);
			break;
		}
		case DrawShapes::DRAW_Polygon:
		{
			QPolygon pts(polygon.size());
			for (int i = 0; i < polygon.size(); i++) {
				pts.putPoints(i, 1, polygon[i].x, polygon[i].y);
			}
			pts.putPoints(polygon.size(), 1, polygon[0].x, polygon[0].y);
			painter->drawPolygon(pts);
			break;
		}
		default:
			break;
		}
	}
}

void ToolDrawPolygon::gl_draw() {
	if (!isActivate) {
		return;
	}
}
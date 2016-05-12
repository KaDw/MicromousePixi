#pragma once
#include "stdafx.h"
#include "controlWQ.h"
#include "button.h"
#include "lab.h"

using namespace sf;
class GUI
{
	static GUI* _instance;
	Font _font;
	uint8_t _fontSize = 22;
	std::map<std::string, button*> _btnMap;
	sf::Vector2f _btnPos = Vector2f(500, 50);
	Vector2f _cellSize = Vector2f(36, 36);
	float _lab_offset = 4;
	std::thread buttons_thread;


public:
	static GUI* getGUI();
	~GUI();

	void start();
	void handleEvent();
	void addButton(std::string str, btn_cb cb);

	void update();
	void updateButtons(sf::Event e);


	void draw();
	void drawCell(int x, int y);
	void drawMouse(int x, int y, Dir_t dir);
	void drawCost();
	void drawParent();
	void drawMove();
	void drawButtons();
	void drawCellText(int x, int y, const String str, Color col = Color::Black);

	Vector2f getLabCellPos(int x, int y);
	Rect<float> getWallRect(int x, int y, Wall_t w);

	bool isMouseOnWall(int x, int y, Wall_t w);
	void mouseHover();

	void loadLab(const std::string& path);
	void saveLab(const std::string& path);

	bool _drawCost{false};
	bool _drawParent{false};
	bool _drawMove{true};
	bool _drawVisited{false};
	bool _singleStep{false};
	RenderWindow _wnd;
private:
	GUI();
	void handleMouse(Event& e);
};


// GUI Lab
void		GLabInit();
Cell_t*		GLabGetCell(int8_t x, int8_t y);
uint8_t		GLabIsWall(int8_t x, int8_t y, Wall_t w);
void		GLabSetWall(int8_t x, int8_t y, Wall_t w);
void		GLabClearWall(int8_t x, int8_t y, Wall_t w);
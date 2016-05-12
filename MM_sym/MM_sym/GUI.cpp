#include "stdafx.h"
#include "GUI.h"

GUI* GUI::_instance = NULL;
Cell_t gui_Map[LAB_SIZE][LAB_SIZE];


extern LCoord_t g_CurrX;
extern LCoord_t g_CurrY;
extern LCoord_t g_CurrTarX;
extern LCoord_t g_CurrTarY;
extern Dir_t g_CurrDir;
extern Mode_t g_Mode;

void cb(button& btn);
void cb_cost(button& btn);
void cb_move(button& btn);
void cb_parent(button& btn);
void cb_flood(button& btn);
void cb_save(button& btn);
void cb_load(button& btn);
void cb_reset(button& btn);

using std::cout;
using std::thread;
using std::endl;

void SensorUpdateSWD()
{
	// gdy to bedzie na rawde pobierane z sensorow
	// to bedzie wygladalo zupelnie inaczej!
	Dir_t dir;
	LCoord_t x, y;
	Wall_t wall;

	// wyczysc sensory
	g_SWD = 0;

	// dla scian z przodu
	dir = DIR_N + g_CurrDir;
	dir %= DIR_MODULO;
	wall = WALL_N;
	wall <<= (g_CurrDir >> 1);
	if (wall > WALL_W)
		wall >>= 4;
	assert(wall <= WALL_W);

	x = g_CurrX + LabGetDeltaX(dir);
	y = g_CurrY + LabGetDeltaY(dir);

	if (GLabIsWall(g_CurrX, g_CurrY, wall))
		g_SWD |= SWD_FORWARD;
	else
	{
		if (GLabIsWall(x, y, wall))
			g_SWD |= SWD_FAR_FORWARD;

		// dla scian po skosie
		// kierunek jest ten sam co dla tych do przodu


		if (g_SWD & SWD_CROSS_LEFT)
		{
			wall = WALL_E << (g_CurrDir >> 1);
			if (wall > WALL_W)
				wall >>= 4;
			assert(wall <= WALL_W);

			LabSetWall(x, y, wall);
		}

		if (g_SWD & SWD_CROSS_RIGHT)
		{
			wall = WALL_W << (g_CurrDir >> 1);
			if (wall > WALL_W)
				wall >>= 4;
			assert(wall <= WALL_W);

			LabSetWall(x, y, wall);
		}
	}

	// dla scian z lewej
	dir = DIR_W + g_CurrDir;
	dir %= DIR_MODULO;
	wall = WALL_W;
	wall <<= (g_CurrDir >> 1);
	if (wall > WALL_W)
		wall >>= 4;
	assert(wall <= WALL_W);

	x = g_CurrX + LabGetDeltaX(dir);
	y = g_CurrY + LabGetDeltaY(dir);

	if (GLabIsWall(g_CurrX, g_CurrY, wall))
		g_SWD |= SWD_LEFT;
	else if (GLabIsWall(x, y, wall))
		g_SWD |= SWD_FAR_LEFT;


	// dla scian z prawej
	dir = DIR_E + g_CurrDir;
	dir %= DIR_MODULO;
	wall = WALL_E;
	wall <<= (g_CurrDir >> 1);
	if (wall > WALL_W)
		wall >>= 4;
	assert(wall <= WALL_W);

	x = g_CurrX + LabGetDeltaX(dir);
	y = g_CurrY + LabGetDeltaY(dir);

	if (GLabIsWall(g_CurrX, g_CurrY, wall))
		g_SWD |= SWD_RIGHT;
	else if (GLabIsWall(x, y, wall))
		g_SWD |= SWD_FAR_RIGHT;
}


void cb(button& btn)
{
	list_s* l{ ConGetMoves() };
	Move_t move = 0;
	static Move_t lastMove = 0;

	// pobierz i usun pierwszy ruch do wykonania
	if (!list_empty(l))
	{
		move = list_front(l);
		list_pop_front(l);
	}

	g_CurrX += (move == MOVE_R) - (move == MOVE_L);
	g_CurrY += (move == MOVE_F) - (move == MOVE_B);
	g_CurrDir = (g_CurrDir + ConConvertMoveLoc(lastMove, move) * 2) % 8;
	lastMove = move;

	cb_flood(btn);
}

void cb_cost(button& btn)
{
	GUI* g = GUI::getGUI();
	g->_drawCost = !g->_drawCost;
	
	if (g->_drawCost == true)
		g->_drawMove = false;
}

void cb_move(button& btn)
{
	GUI* g = GUI::getGUI();
	g->_drawMove = !g->_drawMove;

	if (g->_drawMove == true)
		g->_drawCost = false;
}

void cb_parent(button& btn)
{
	GUI* g = GUI::getGUI();
	g->_drawParent = !g->_drawParent;
}

void printTime(std::string text, std::chrono::high_resolution_clock::time_point& t1, std::chrono::high_resolution_clock::time_point& t2)
{
	std::cout << text << std::setw(7) << abs(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()) << "ms ";
}

void cb_floodRaw()
{
	static long fulltime = 0;
	static int stepnum = 1;
	typedef std::chrono::high_resolution_clock Clock;


	LabSetVisited(g_CurrX, g_CurrY);
	auto t0 = Clock::now();
	SensorUpdateSWD();
	auto t1 = Clock::now();
	//ConUpdateLabFromCenter();
	auto t2 = Clock::now();


	// change mode if it is time to do that
	if (g_Mode == MODE_TO_TARGET && g_CurrX == g_TargetX && g_CurrY == g_TargetY)
	{
		g_Mode = MODE_TO_START;
		g_CurrTarX = g_TargetX;
		g_CurrTarY = g_TargetY;
	}
	else if (g_Mode == MODE_TO_START && g_CurrX == 0 && g_CurrY == 0)
	{
		g_Mode = MODE_FAST_TARGET;
		g_CurrTarX = 0;
		g_CurrTarY = 0;
	}
	else if (g_Mode == MODE_FAST_TARGET && g_CurrX == g_TargetX && g_CurrY == g_TargetY)
	{
		g_Mode = MODE_FAST_START;
		g_CurrTarX = g_TargetX;
		g_CurrTarY = g_TargetY;
	}
	else if (g_Mode == MODE_FAST_START && g_CurrX == 0 && g_CurrY == 0)
	{
		g_Mode = MODE_STOP;
		g_CurrTarX = 0;
		g_CurrTarY = 0;
	}

	auto t3 = Clock::now();
	auto t4 = t3, t5 = t3;

	
	ConFloodTo();
	t4 = Clock::now();
	//ConFindPathByCostTo();
	t5 = Clock::now();

	if (g_Mode == MODE_STOP)
	{
		printf("Zatrzymaj sie!! file:%s  line:%d\n", __FILE__, __LINE__);
	}

	fulltime += abs(std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count());
	printf("%d. ", stepnum);
	printTime("Flood ", t3, t4);
	printTime("   Path ", t4, t5);
	printf("   fullFlood: %d  avegrade: %d\n", fulltime, fulltime/stepnum);
	++stepnum;
}

std::thread flood_thread;

void cb_flood(button& btn)
{
	if (flood_thread.joinable())
		flood_thread.join();

	flood_thread = std::thread(cb_floodRaw);
	//cb_floodRaw();

}

void cb_save(button& btn)
{
	std::string s;

	std::cout << "\nJaka ma byc nazwa lab?   ";
	std::cin >> s;

	// usun rozszerzenie
	//pos = s.rfind('.');
	//s.erase(pos, s.length() - pos);

	//printf("\nPo usunieciu '.' jest: %s", s.c_str());

	GUI* g = GUI::getGUI();
	g->saveLab(s);
}

void cb_load(button& btn)
{
	std::string s;

	std::cout << "\nJaki labirynt wczytac?  ";
	std::cin >> s;
	
	// usun rozszerzenie
	//pos = s.rfind('.');
	//s.erase(pos, s.length() - pos);

	//printf("\nPo usunieciu '.' jest: %s", s.c_str());
	
	GUI* g = GUI::getGUI();
	g->loadLab(s);
}

void cb_reset(button& btn)
{
	// clear visited - to show path (temporary)
	for (int i = 0; i < LAB_SIZE*LAB_SIZE; ++i)
	{
		LabClearVisited(i / LAB_SIZE, i % LAB_SIZE);
		LabClearWall(i / LAB_SIZE, i% LAB_SIZE, WALL_MASK);
	}
	ConInit();
}


GUI::GUI() : _wnd(sf::VideoMode(600, 600), "Elo")
{
	if (!_font.loadFromFile("arial.ttf"))
		throw std::exception("B³¹d ³adowania Czcionki");

	//_btn.setSize(15);
	addButton("Step", cb);
	addButton("Cost", cb_cost);
	addButton("Move", cb_move);
	addButton("Parent", cb_parent);
	addButton("Flood", cb_flood);
	addButton("Save lab", cb_save);
	addButton("Load lab", cb_load);
	addButton("Reset", cb_reset);

	GLabInit();
	loadLab("default.lab");
	ConInit();
	ConNextStep();
	_wnd.setFramerateLimit(40);
}


GUI * GUI::getGUI()
{
	if (_instance == NULL)
		_instance = new GUI();

	return _instance;
}

GUI::~GUI()
{
	std::map<std::string, button*>::iterator ib;

	for (ib = _btnMap.begin(); ib != _btnMap.end(); ++ib)
	{
		delete ib->second;
	}

	if (flood_thread.joinable())
		flood_thread.join();

	if (buttons_thread.joinable())
		buttons_thread.join();
}



void GUI::handleEvent()
{
	sf::Event event;
	std::stringstream ss;

	while (_wnd.pollEvent(event))
	{
		switch (event.type)
		{
		case(sf::Event::Closed) :
			_wnd.close();
			break;
		case sf::Event::MouseMoved:
			ss.str("");
			ss << std::setw(4) << sf::Mouse::getPosition(_wnd).x << "_" << std::setw(4) << sf::Mouse::getPosition(_wnd).y;
			//_wnd.setTitle(sf::String(ss.str()));
			break;
		default:
			break;
		}

		updateButtons(event);
		handleMouse(event);
	}
}


void GUI::update()
{
	_wnd.clear(sf::Color(240, 240, 245));
	handleEvent();
	draw();
	_wnd.display();
	sleep(sf::milliseconds(3));
}

void GUI::updateButtons(sf::Event e)
{
	std::map<std::string, button*>::iterator ib;

	for (ib = _btnMap.begin(); ib != _btnMap.end(); ++ib)
	{
		ib->second->update(e, _wnd);
	}
}

void GUI::addButton(std::string str, btn_cb callback)
{
	//sf::Vector2f maxDim;
	sf::Vector2f pos = _btnPos;
	pos.y += _btnMap.size() * _fontSize * 2;
	button* btn = new button(str, _font, pos, callback);
	_btnMap.insert(std::pair<std::string, button*>(str, btn));

	/*maxDim = btn->getDimensions();

	for (auto i{ _btnMap.begin() }; i != _btnMap.end(); ++i)
		if (maxDim.x < (i->second)->getDimensions().x)
			maxDim.x = (i->second)->getDimensions().x;

	for (auto& button : _btnMap)
		(button.second)->setDimension(maxDim);*/
}


void GUI::drawCell(int x, int y)
{
	Vector2f pos = getLabCellPos(x, y);
	sf::RectangleShape Lbox(_cellSize); // large box (background)
	sf::RectangleShape Sbox(sf::Vector2f(_lab_offset, _lab_offset)); //small box (walls, dots)
	uint8_t(*chackWall)(int8_t x, int8_t y, Wall_t w);

	if (LabIsVisited(x, y))
		Lbox.setFillColor(sf::Color::Green);
	/*else if (LabIsBlind(x, y))
		Lbox.setFillColor(sf::Color::Red);*/
	else
		Lbox.setFillColor(sf::Color(180, 180, 255)); //blue

	// dot color
	Sbox.setFillColor(sf::Color(50, 10, 60)); 

	// draw background and dots
	Lbox.setPosition(pos);
	_wnd.draw(Lbox);
	Sbox.setPosition(pos.x + _cellSize.x, pos.y + _cellSize.y);
	_wnd.draw(Sbox);
	Sbox.setPosition(pos.x + _cellSize.x, pos.y - _lab_offset);
	_wnd.draw(Sbox);
	Sbox.setPosition(pos.x - _lab_offset, pos.y + _cellSize.y);
	_wnd.draw(Sbox);
	Sbox.setPosition(pos.x - _lab_offset, pos.y - _lab_offset);
	_wnd.draw(Sbox);


	// draw lab walls (from GUI map)

	// wall color
	Sbox.setFillColor(Color(15, 50, 20, 80));

	// draw walls
	if (GLabIsWall(x, y, WALL_N))
	{
		Sbox.setSize(Vector2f(_cellSize.x, _lab_offset));
		Sbox.setPosition(Vector2f(pos.x, pos.y - _lab_offset));
		_wnd.draw(Sbox);
	}
	if (GLabIsWall(x, y, WALL_S))
	{
		Sbox.setSize(Vector2f(_cellSize.x, _lab_offset));
		Sbox.setPosition(Vector2f(pos.x, pos.y + _cellSize.y));
		_wnd.draw(Sbox);
	}
	if (GLabIsWall(x, y, WALL_W))
	{
		Sbox.setSize(Vector2f(_lab_offset, _cellSize.y));
		Sbox.setPosition(Vector2f(pos.x - _lab_offset, pos.y));
		_wnd.draw(Sbox);
	}
	if (GLabIsWall(x, y, WALL_E))
	{
		Sbox.setSize(Vector2f(_lab_offset, _cellSize.y));
		Sbox.setPosition(Vector2f(pos.x + _cellSize.x, pos.y));
		_wnd.draw(Sbox);
	}


	// choose lab to draw
	if (g_CurrX == 0 && g_CurrY == 0)
		chackWall = GLabIsWall;
	else
		chackWall = LabIsWall;

	// wall color
	Sbox.setFillColor(Color(15, 50, 20));

	// draw walls
	if (chackWall(x,y,WALL_N))
	{
		Sbox.setSize(Vector2f(_cellSize.x, _lab_offset));
		Sbox.setPosition(Vector2f(pos.x, pos.y - _lab_offset));
		_wnd.draw(Sbox);
	}
	if (chackWall(x, y, WALL_S))
	{
		Sbox.setSize(Vector2f(_cellSize.x, _lab_offset));
		Sbox.setPosition(Vector2f(pos.x, pos.y + _cellSize.y));
		_wnd.draw(Sbox);
	}
	if (chackWall(x, y, WALL_W))
	{
		Sbox.setSize(Vector2f(_lab_offset, _cellSize.y));
		Sbox.setPosition(Vector2f(pos.x - _lab_offset, pos.y));
		_wnd.draw(Sbox);
	}
	if (chackWall(x, y, WALL_E))
	{
		Sbox.setSize(Vector2f(_lab_offset, _cellSize.y));
		Sbox.setPosition(Vector2f(pos.x + _cellSize.x, pos.y));
		_wnd.draw(Sbox);
	}
}

void GUI::drawMouse(int x, int y, Dir_t dir)
{
	//RectangleShape point = RectangleShape(Vector2f(_lab_offset, _lab_offset));
	//point.setFillColor(Color::Red);
	//point.setPosition(getLabCellPos(x, y));
	CircleShape triangle = CircleShape(_cellSize.x*0.5f, 3);
	triangle.setOrigin(_cellSize.x*0.5f, _cellSize.x*0.5f);
	triangle.scale(1.f, 0.5f);
	triangle.rotate(static_cast<float>(dir * 45));
	triangle.setPosition(getLabCellPos(x, y));
	triangle.move(_cellSize.x*0.5f, _cellSize.y*0.5f);
	triangle.setFillColor(Color(145, 86, 32, 200));

	_wnd.draw(triangle);
	//_wnd.draw(point);
}

void GUI::drawCost()
{
	if (_drawCost == false)
		return;

	Cost_t c;
	std::stringstream ss;
	for (int x = 0; x < LAB_SIZE; ++x)
		for (int y = 0; y < LAB_SIZE; ++y)
		{
			ss.str("");
			c = LabGetCostReal(x, y);// +LabGetCostImag(x, y);

			if (c < 999 && c < MAX_COST)
				ss << c;
			else
				continue;

			drawCellText(x, y, ss.str());
		}
}

void GUI::drawParent()
{

	if (_drawParent == false)
		return;

	sf::RectangleShape rect(Vector2f(_cellSize.x*0.1f, _cellSize.y*0.5f));
	sf::Vector2f off;

	rect.setFillColor(Color(250, 50, 50, 150));
	rect.setOrigin(rect.getGlobalBounds().width*0.5f, rect.getGlobalBounds().height*0.9f);
	off = _cellSize*0.5f;

	for (int x = 0; x < LAB_SIZE; ++x)
		for (int y = 0; y < LAB_SIZE; ++y)
		{
			rect.setRotation((LabGetParent(x, y) >> 4)*90.f);
			rect.setPosition(getLabCellPos(x, y) + off);
			_wnd.draw(rect);
		}
}

void GUI::drawMove()
{
	Move_t lastMove = MOVE_F;
	Dir_t rot;
	std::map<Move_t, const char*> moveToStr;

	if (_drawMove == false)
		return;

	moveToStr[MOVE_F] = "F";
	moveToStr[MOVE_R] = "R";
	moveToStr[MOVE_B] = "B";
	moveToStr[MOVE_L] = "L";

	int x = ConGetX();
	int y = ConGetY();
	list_s* li = ConGetMoves();
	list_iterator it = list_begin(li);

	int temp = 0;

	for (it = list_begin(li); it != list_end(li); it = list_next(li, it))
	{
		++temp;
		rot = ConConvertMoveLoc(lastMove, *it);
		lastMove = *it;
		if(LabIsIn(x, y))
			drawCellText(x, y, moveToStr[rot]);
		x += (*it == MOVE_R) - (*it == MOVE_L);
		y += (*it == MOVE_F) - (*it == MOVE_B);

		if (!LabIsIn(x, y))
		{
			// gdy cos jest nie tak, to pokaz w konsoli gdzie chciales jechac
			// ale nie pokazuj tego w GUI, aby nie bylo bledow
			printf("\nMove:");
			while (it != list_end(li))
			{
				printf(" %c", *(moveToStr[*it]));
				it = list_next(li, it);
			}
			return;
		}
	}
}


void GUI::drawButtons()
{
	std::map<std::string, button*>::iterator ib;

	for (ib = _btnMap.begin(); ib != _btnMap.end(); ++ib)
	{
		_wnd.draw(*(ib->second));
	}
}


void GUI::drawCellText(int x, int y, const String str, Color col)
{
	Text txt = Text(str, _font, _fontSize);
	Vector2f off = Vector2f(txt.getLocalBounds().width*0.5f, txt.getLocalBounds().height*0.5f);
	Vector2f pos = getLabCellPos(x, y);
	pos.x += _cellSize.x*0.5f;
	pos.x -= off.x;
	txt.setColor(col);
	txt.setPosition(pos);
	_wnd.draw(txt);
}


void GUI::draw()
{

	// draw lab background
	_wnd.clear(sf::Color(240, 240, 245));

	mouseHover();

	for (int x = 0; x < LAB_SIZE; ++x)
		for (int y = 0; y < LAB_SIZE; ++y)
		{
			drawCell(x, y);
		}

	drawMouse(ConGetX(), ConGetY(), ConGetMouseDir());
	drawParent();
	drawCost();
	drawMove();
	drawButtons();
	_wnd.display();
}


Vector2f GUI::getLabCellPos(int x, int y)
{
	assert_xy(x, y);

	Vector2f off = Vector2f(25, 15);
	return Vector2f(off.x + static_cast<float>(x * (_cellSize.x + _lab_offset)), off.y + static_cast<float>((LAB_SIZE - y - 1) * (_cellSize.y + _lab_offset)) );
}


Rect<float> GUI::getWallRect(int x, int y, Wall_t w)
{
	assert(0 <= x && 0 <= y);
	assert(x <= LAB_SIZE && y <= LAB_SIZE);
	assert(0 < (int)w && (int)w <= 8);

	Vector2f wsp;
	Rect<float> rec;
	wsp = getLabCellPos(x, y);
	rec.left = wsp.x;
	rec.top = wsp.y;

	switch (w)
	{
	case WALL_N:
		rec.top -= _lab_offset;
		rec.width = _cellSize.x;
		rec.height = _lab_offset;
		break;
	case WALL_S:
		rec.top += _cellSize.y;
		rec.width = _cellSize.x;
		rec.height = _lab_offset;
		break;
	case WALL_W:
		rec.left -= _lab_offset;
		rec.width = _lab_offset;
		rec.height = _cellSize.y;
		break;
	case WALL_E:
		rec.left += _cellSize.x;
		rec.width = _lab_offset;
		rec.height = _cellSize.y;
		break;
	default:
		throw std::exception("isMouseOnWall: bad 3 argument");
		break;
	}

	return rec;
}


bool GUI::isMouseOnWall(int x, int y, Wall_t w)
{
	assert(0 <= x && 0 <= y);
	assert(x <= LAB_SIZE && y <= LAB_SIZE);
	assert(0 < (int)w && (int)w <= 8);

	Rect<float> r = getWallRect(x, y, w);
	/*RectangleShape rs;
	rs.setFillColor(Color::Blue);
	rs.setPosition(r.left, r.top);
	rs.setSize(Vector2f(r.width, r.height));
	_wnd.draw(rs);*/
	return r.contains(static_cast<float>(Mouse::getPosition(_wnd).x), static_cast<float>(Mouse::getPosition(_wnd).y));
}


void GUI::mouseHover()
{
	Vector2f wsp;
	Rect<float> rec;
	RectangleShape rs;

	for (int x = 0; x < LAB_SIZE; ++x)
		for (int y = 0; y < LAB_SIZE; ++y)
			for (Wall_t w = (Wall_t)1; w <= 8; w = (Wall_t)((int)w << 1))
			{
				if (isMouseOnWall(x, y, w))
				{
					rec = getWallRect(x, y, w);
					rs.setFillColor(Color(110, 110, 110));
					rs.setPosition(rec.left, rec.top);
					rs.setSize(Vector2f(rec.width, rec.height));
					_wnd.draw(rs);
				}
			}
}


void GUI::handleMouse(Event& e)
{
	Vector2f wsp;
	Rect<float> rec;
	RectangleShape rs;

	if (e.type != sf::Event::MouseButtonReleased || e.mouseButton.button != Mouse::Left)
		return;

	for (int x = 0; x < LAB_SIZE; ++x)
		for (int y = 0; y < LAB_SIZE; ++y)
			for (Wall_t w = (Wall_t)1; w <= 8; w = (Wall_t)((int)w << 1))
			{
				if (isMouseOnWall(x, y, w))
				{
					if (GLabIsWall(x, y, w))
						GLabClearWall(x, y, w);
					else
						GLabSetWall(x, y, w);

					return;
				}
			}
}

void GUI::loadLab(const std::string & path)
{
	try
	{
		int c;
		std::ifstream ifs(path, std::ofstream::in);

		if (!ifs.is_open())
			throw std::exception("Blad podczas otwierania pliku");

		for (auto x = 0; x < LAB_SIZE; ++x)
			for (auto y = 0; y < LAB_SIZE; ++y)
			{
				ifs >> c;
				GLabSetWall(x, y, c);
			}

		printf("Wczytano labirynt %s\n", path.c_str());
		_wnd.setTitle("Labirynt: " + path);
	}
	catch (...)
	{
		printf("\nOdczyt sie chyba nie udal...\n");
	}
}

void GUI::saveLab(const std::string & path)
{
	try
	{
		if(g_CurrX != 0 || g_CurrY != 0)
			throw std::exception("Ruszyles sie, teraz nie mozesz zapisywac labiryntu do pliku");

		std::ofstream ofs(path, std::ofstream::out | std::ofstream::trunc);

		if (!ofs.is_open())
			throw std::exception("Blad podczas otwierania pliku");

		for (auto x = 0; x < LAB_SIZE; ++x)
		{
			for (auto y = 0; y < LAB_SIZE; ++y)
			{
				ofs << (*GLabGetCell(x, y) & WALL_MASK) << ' ';
			}
			ofs << std::endl;
		}

		printf("Zapisano jako %s\n", path.c_str());
	}
	catch (const std::exception& e)
	{
		printf("%s", e.what());
	}
	catch (...)
	{
		printf("\nZapis sie chyba nie udal...\n");
	}
}


void draw_thread()
{
	GUI* g = GUI::getGUI();

	while (1)
	{
		printf("d");
		if (!g->_wnd.isOpen())
			break;
		g->draw();
		std::this_thread::sleep_for(std::chrono::microseconds(30));
	}
}

void handle_thread()
{
	GUI* g = GUI::getGUI();

	while (1)
	{
		printf("h");
		if (!g->_wnd.isOpen())
			break;
		g->handleEvent();
		std::this_thread::sleep_for(std::chrono::microseconds(30));
	}
}


void GUI::start()
{
	//auto handle = std::async(std::launch::async, draw_thread);
	//auto handle2 = std::async(std::launch::async, handle_thread);
	//std::async td([] { GUI*g = GUI::getGUI();  while (1) {g->draw();			sleep(sf::milliseconds(100));} });
	//std::furure<void> th([] { GUI*g = GUI::getGUI();  while (1) {g->handleEvent();	sleep(sf::milliseconds(100));} });
	//std::thread td = std::thread(draw_thread);
	//std::thread th = std::thread(handle_thread);
	//td.detach();
	//th.detach();
	
	//td.join();
	//th.join();

	while (_wnd.isOpen())
	{
		handleEvent();
		draw();
	}

	if (flood_thread.joinable())
		flood_thread.join();

	if (buttons_thread.joinable())
		buttons_thread.join();
}






/////////////////////////
// GUI LAB
/////////////////////////


void GLabInit()
{
	for (LCoord_t x = 0; x < LAB_SIZE; ++x)
	{
		GLabSetWall(x, 0, WALL_S);
		GLabSetWall(x, LAB_SIZE - 1, WALL_N);
	}

	for (uint8_t y = 0; y < LAB_SIZE; ++y)
	{
		GLabSetWall(0, y, WALL_W);
		GLabSetWall(LAB_SIZE - 1, y, WALL_E);
	}
}

void GLabSetWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	w &= 0x0f; // get only wall info

			   // set wall in pointed cell
	gui_Map[x][y] |= w;

	// and in neighbourly cell
	if ((w & WALL_N) && y + 1<LAB_SIZE)
		gui_Map[x][y + 1] |= WALL_S;
	if ((w & WALL_E) && x + 1<LAB_SIZE)
		gui_Map[x + 1][y] |= WALL_W;
	if ((w & WALL_S) && y - 1 >= 0)
		gui_Map[x][y - 1] |= WALL_N;
	if ((w & WALL_W) && x - 1 >= 0)
		gui_Map[x - 1][y] |= WALL_E;
}

void GLabClearWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	w &= 0x0f; // get only wall info

			   // clear wall in pointed cell
	gui_Map[x][y] &= ~w;

	// and in neighbourly cell
	if ((w & WALL_N) && y + 1<LAB_SIZE)
		gui_Map[x][y + 1] &= ~WALL_S;
	if ((w & WALL_E) && x + 1<LAB_SIZE)
		gui_Map[x + 1][y] &= ~WALL_W;
	if ((w & WALL_S) && y - 1 >= 0)
		gui_Map[x][y - 1] &= ~WALL_N;
	if ((w & WALL_W) && x - 1 >= 0)
		gui_Map[x - 1][y] &= ~WALL_E;
}

Cell_t* GLabGetCell(int8_t x, int8_t y)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);

	return &gui_Map[x][y];
}

uint8_t GLabIsWall(int8_t x, int8_t y, Wall_t w)
{
	assert(x >= 0 && y >= 0);
	assert(x < LAB_SIZE && y < LAB_SIZE);
	return gui_Map[x][y] & w;
}
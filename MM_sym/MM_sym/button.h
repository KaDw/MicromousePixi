// sfml button class
// author: Karol Trzciñski
// date 25-03-2016
//
//
// Button class working with SFML library
//
// Brief indtruction:
// 1) create button instance and set text, font, position and callback
// 2) regulary call update() and draw()
//
// when button will be hitted, then callback will run once
// Enjoy!


#pragma once

#include <iostream>
#include <SFML/Graphics.hpp>

class button;


// pointer to function like this:
// void myCallback(button& btn) { }
typedef void(*btn_cb)(button&);

namespace style
{
	enum
	{
		none = 0,
		save = 1,
		cancel = 2,
		clean = 3,
	};
};

namespace state
{
	enum
	{
		normal = 0,
		hovered = 1,
		clicked = 2
	};
};

class button : public sf::Drawable
{
public:
	button();
	button(std::string s, sf::Font& font, sf::Vector2f position, btn_cb cb = NULL);

	~button();

	void create(std::string s, sf::Font& font, sf::Vector2f position, btn_cb call);
	void setCallback(btn_cb callback) { m_callback = callback; }
	void setColorText(sf::Color text) { m_textNormal = text; };
	void setColorTextHover(sf::Color text) { m_textHover = text; };
	void setColorTextClicked(sf::Color text) { m_textClicked = text; };
	void setColorBg(sf::Color bgNormal) { m_bgNormal = bgNormal; };
	void setColorBgHover(sf::Color bgHover) { m_bgHover = bgHover; };
	void setColorCBglicked(sf::Color bgClicked) { m_bgClicked = bgClicked; };
	void setBorderColor(sf::Color border) { m_borderColor = border; };
	void setBorderThickness(float t) { m_borderThickness = t; };
	void setPosition(sf::Vector2f position) { m_position = position; };
	void setSize(unsigned int size);
	void setDimension(const sf::Vector2f&);
	void setText(std::string s) {m_text.setString(s);};
	void setStyle(sf::Uint32 style);
	void setFont(sf::Font& font);

	btn_cb getCallback() const { return m_callback; };
	sf::Vector2f getPosition() { return m_position; };
	sf::Vector2f getDimensions() { return sf::Vector2f(m_button.getGlobalBounds().width, m_button.getGlobalBounds().height); };
	sf::Uint32 getState() { return m_btnstate; };

	virtual void update(sf::Event& e, sf::RenderWindow& window);

protected:
	virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;


private:

	sf::Color m_bgNormal;
	sf::Color m_bgHover;
	sf::Color m_bgClicked;
	sf::Color m_textNormal;
	sf::Color m_textHover;
	sf::Color m_textClicked;
	sf::Color m_borderColor;

	float m_borderThickness;
	float m_borderRadius;
	float m_shadowOff = 1.5f;
	sf::Vector2f m_size;
	sf::Vector2f m_position;
	sf::Uint32 m_style = style::none;
	sf::Uint32 m_btnstate = state::normal;

	sf::RectangleShape m_button;
	sf::RectangleShape m_border;
	sf::Font m_font;
	unsigned int m_fontSize;
	sf::Text m_text;
	sf::Text m_shadow;

	//void (*m_callback)(button&) = 0;
	btn_cb m_callback = 0;
};
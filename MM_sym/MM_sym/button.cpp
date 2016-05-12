#include "stdafx.h"
#include "button.h"

button::button()
{
	m_style = style::none;
}

button::button(std::string s, sf::Font& font, sf::Vector2f position, btn_cb cb)
{
	create(s, font, position, cb);
}

button::~button()
{

}


void button::create(std::string s, sf::Font & font, sf::Vector2f position, btn_cb call)
{
	//set position
	m_position = position;

	//set initial state
	m_btnstate = state::normal;

	//set button style
	setStyle(style::none);
	setSize(24);

	//callback
	setCallback(call);

	//set up text
	m_text.setString(s);
	m_text.setFont(font);
	m_text.setOrigin(m_text.getGlobalBounds().width / 2, m_text.getGlobalBounds().height / 2);
	m_text.setColor(m_textNormal);

	//set some defauts
	m_borderRadius = 5.f;
	m_borderThickness = 0.f;
	m_size = sf::Vector2f(m_text.getGlobalBounds().width * 1.5f, m_text.getGlobalBounds().height * 1.5f);

	//m_button = thor::Shapes::roundedRect(m_size, m_borderRadius, m_bgNormal, m_borderThickness, m_border);
	m_button = sf::RectangleShape(m_size);
	m_button.setOrigin(m_button.getGlobalBounds().width / 2, m_button.getGlobalBounds().height / 2);
	m_button.setPosition(m_position);

	sf::Vector2f textPosition = sf::Vector2f(m_button.getPosition().x, m_button.getPosition().y - m_button.getGlobalBounds().height / 4);

	m_text.setPosition(textPosition);

	m_shadow.setFont(font);
	m_shadow = m_text;
	m_shadow.setOrigin(m_shadow.getGlobalBounds().width / 2, m_shadow.getGlobalBounds().height / 2);
	m_shadow.setPosition(m_text.getPosition().x + 0.3f, m_text.getPosition().y + 0.0f);
}

void button::setSize(unsigned int size)
{
	m_fontSize = size;
	m_text.setCharacterSize(m_fontSize);
	m_text.setOrigin(m_text.getGlobalBounds().width / 2, m_text.getGlobalBounds().height / 2);
	m_shadow.setCharacterSize(m_fontSize);
	m_shadow.setOrigin(m_shadow.getGlobalBounds().width / 2, m_shadow.getGlobalBounds().height / 2);
	m_size = sf::Vector2f(m_text.getGlobalBounds().width * 1.5f, (m_text.getGlobalBounds().height + m_text.getGlobalBounds().height) * 1.5f);
	//m_button = thor::Shapes::roundedRect(m_size, m_borderRadius, m_bgNormal, m_borderThickness, m_border);
	m_button = sf::RectangleShape(m_size);
}

void button::setDimension(const sf::Vector2f & dim)
{
	m_size = dim;
}

void button::setStyle(sf::Uint32 style)
{
	//set button style
	m_style = style;

	switch (m_style)
	{
	case style::none:
	{
		m_textNormal = sf::Color(240, 230, 210);
		m_textHover = sf::Color(0, 0, 0);
		m_textClicked = sf::Color(100, 100, 100, 200);
		m_bgNormal = sf::Color(141, 3, 3);
		m_bgHover = sf::Color(140, 50, 50, 240);
		m_bgClicked = sf::Color(180, 180, 180);
		m_borderColor = sf::Color(0, 0, 0, 100);
	}
	break;

	case style::save:
	{
		m_textNormal = sf::Color(255, 255, 255);
		m_textHover = sf::Color(255, 255, 255);
		m_textClicked = sf::Color(255, 255, 255);
		m_bgNormal = sf::Color(0, 255, 0, 100);
		m_bgHover = sf::Color(0, 200, 0, 100);
		m_bgClicked = sf::Color(0, 150, 0);
		m_borderColor = sf::Color(0, 0, 0, 100);
	}
	break;

	case style::cancel:
	{
		m_textNormal = sf::Color(255, 255, 255);
		m_textHover = sf::Color(255, 255, 255);
		m_textClicked = sf::Color(255, 255, 255);
		m_bgNormal = sf::Color(255, 0, 0, 100);
		m_bgHover = sf::Color(200, 0, 0, 100);
		m_bgClicked = sf::Color(150, 0, 0);
		m_borderColor = sf::Color(255, 255, 255, 100);
	}
	break;

	case style::clean:
	{
		m_textNormal = sf::Color(255, 255, 255);
		m_textHover = sf::Color(255, 255, 255);
		m_textClicked = sf::Color(255, 255, 255);
		m_bgNormal = sf::Color(0, 255, 200, 100);
		m_bgHover = sf::Color(0, 200, 200, 100);
		m_bgClicked = sf::Color(0, 150, 150);
		m_borderColor = sf::Color(255, 255, 255, 100);
	}
	break;

	default:
		break;
	}
}

void button::setFont(sf::Font& font)
{
	m_text.setFont(font);
	m_shadow.setFont(font);
}

void button::update(sf::Event& e, sf::RenderWindow& window)
{
	//perform updates for settings from user
	switch (m_style)
	{
	case style::none:
	{
		m_size = sf::Vector2f(m_text.getGlobalBounds().width * 1.5f, m_text.getGlobalBounds().height * 1.75f);
		//m_button = thor::Shapes::roundedRect(m_size, m_borderRadius, m_bgNormal, m_borderThickness, m_border);
		m_button = sf::RectangleShape(m_size);
		m_button.setOrigin(m_button.getGlobalBounds().width / 2, m_button.getGlobalBounds().height / 2);
		m_button.setPosition(m_position);
		m_text.setOrigin(m_text.getGlobalBounds().width / 2, m_text.getGlobalBounds().height / 2);
		sf::Vector2f textPosition = sf::Vector2f(m_button.getPosition().x, m_button.getPosition().y - m_button.getGlobalBounds().height / 4);
		m_text.setPosition(textPosition);
		m_text.setColor(m_textNormal);
		m_shadow.setOrigin(m_shadow.getGlobalBounds().width / 2, m_shadow.getGlobalBounds().height / 2);
		m_shadow.setPosition(m_text.getPosition().x + m_shadowOff, m_text.getPosition().y + m_shadowOff);
		m_shadow.setColor(sf::Color(0, 0, 0, 100));
	}
	break;

	case style::save:
	{
		m_size = sf::Vector2f(m_text.getGlobalBounds().width * 1.5f, m_text.getGlobalBounds().height * 1.75f);
		//m_button = thor::Shapes::roundedRect(m_size, m_borderRadius, m_bgNormal, m_borderThickness, m_border);
		m_button = sf::RectangleShape(m_size);
		m_button.setOrigin(m_button.getGlobalBounds().width / 2, m_button.getGlobalBounds().height / 2);
		m_button.setPosition(m_position);
		m_text.setOrigin(m_text.getGlobalBounds().width / 2, m_text.getGlobalBounds().height / 2);
		sf::Vector2f textPosition = sf::Vector2f(m_button.getPosition().x, m_button.getPosition().y - m_button.getGlobalBounds().height / 4);
		m_text.setPosition(textPosition);
		m_text.setColor(m_textNormal);
		m_shadow.setOrigin(m_shadow.getGlobalBounds().width / 2, m_shadow.getGlobalBounds().height / 2);
		m_shadow.setPosition(m_text.getPosition().x + m_shadowOff, m_text.getPosition().y + m_shadowOff);
		m_shadow.setColor(sf::Color(0, 0, 0));
	}
	break;

	case style::cancel:
	{
		m_size = sf::Vector2f(m_text.getGlobalBounds().width * 1.5f, m_text.getGlobalBounds().height * 1.75f);
		//m_button = thor::Shapes::roundedRect(m_size, m_borderRadius, m_bgNormal, m_borderThickness, m_border);
		m_button = sf::RectangleShape(m_size);
		m_button.setOrigin(m_button.getGlobalBounds().width / 2, m_button.getGlobalBounds().height / 2);
		m_button.setPosition(m_position);
		m_text.setOrigin(m_text.getGlobalBounds().width / 2, m_text.getGlobalBounds().height / 2);
		sf::Vector2f textPosition = sf::Vector2f(m_button.getPosition().x, m_button.getPosition().y - m_button.getGlobalBounds().height / 4);
		m_text.setPosition(textPosition);
		m_text.setColor(m_textNormal);
		m_shadow.setOrigin(m_shadow.getGlobalBounds().width / 2, m_shadow.getGlobalBounds().height / 2);
		m_shadow.setPosition(m_text.getPosition().x + m_shadowOff, m_text.getPosition().y + m_shadowOff);
		m_shadow.setColor(sf::Color(0, 0, 0));
	}
	break;

	case style::clean:
	{
		m_size = sf::Vector2f(m_text.getGlobalBounds().width * 1.5f, m_text.getGlobalBounds().height * 1.75f);
		//m_button = thor::Shapes::roundedRect(m_size, m_borderRadius, m_bgNormal, m_borderThickness, m_border);
		m_button = sf::RectangleShape(m_size);
		m_button.setOrigin(m_button.getGlobalBounds().width / 2, m_button.getGlobalBounds().height / 2);
		m_button.setPosition(m_position);
		m_text.setOrigin(m_text.getGlobalBounds().width / 2, m_text.getGlobalBounds().height / 2);
		sf::Vector2f textPosition = sf::Vector2f(m_button.getPosition().x, m_button.getPosition().y - m_button.getGlobalBounds().height / 4);
		m_text.setPosition(textPosition);
		m_text.setColor(m_textNormal);
		m_shadow.setOrigin(m_shadow.getGlobalBounds().width / 2, m_shadow.getGlobalBounds().height / 2);
		m_shadow.setPosition(m_text.getPosition().x + m_shadowOff, m_text.getPosition().y + m_shadowOff);
		m_shadow.setColor(sf::Color(0, 0, 0));
	}
	break;

	default:
		break;
	}

	m_border.setFillColor(m_borderColor);
	m_border.setSize(sf::Vector2f(2 * m_borderThickness + m_button.getSize().x, 2 * m_borderThickness + m_button.getSize().y));
	m_border.setOrigin(m_border.getGlobalBounds().width*0.5f, m_border.getGlobalBounds().height*0.5f);
	m_border.setPosition(m_button.getPosition().x-m_borderThickness, m_button.getPosition().y-m_borderThickness);


	//perform updates for user mouse interactions
	sf::Vector2i m_mousePosition = sf::Mouse::getPosition(window);

	bool mouseInButton = m_mousePosition.x >= m_button.getPosition().x - m_button.getGlobalBounds().width / 2
		&& m_mousePosition.x <= m_button.getPosition().x + m_button.getGlobalBounds().width / 2
		&& m_mousePosition.y >= m_button.getPosition().y - m_button.getGlobalBounds().height / 2
		&& m_mousePosition.y <= m_button.getPosition().y + m_button.getGlobalBounds().height / 2;

	if (e.type == sf::Event::MouseMoved)
	{
		if (mouseInButton)
		{
			m_btnstate = state::hovered;
		}

		else
		{
			m_btnstate = state::normal;
		}
	}

	if (e.type == sf::Event::MouseButtonPressed)
	{
		switch (e.mouseButton.button)
		{
		case sf::Mouse::Left:
		{
			if (mouseInButton)
			{
				m_button.setFillColor(m_bgClicked);
				m_text.setColor(m_textClicked);
				m_btnstate = state::clicked;

				if (m_callback != NULL)
					m_callback(*this);
			}

			else
			{
				m_btnstate = state::normal;
			}
		}
		break;
		}
	}

	if (e.type == sf::Event::MouseButtonReleased)
	{
		switch (e.mouseButton.button)
		{
		case sf::Mouse::Left:
		{
			if (mouseInButton)
			{
				m_btnstate = state::hovered;
			}

			else
			{
				m_btnstate = state::normal;
			}
		}
		}
	}

	switch (m_btnstate)
	{
	case state::normal:
	{
		m_button.setFillColor(m_bgNormal);
		m_text.setColor(m_textNormal);
	}
	break;

	case state::hovered:
	{
		m_button.setFillColor(m_bgHover);
		m_text.setColor(m_textHover);
	}
	break;
	}
}

void button::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	
	target.draw(m_border, states);
	target.draw(m_button, states);
	target.draw(m_shadow, states);
	target.draw(m_text, states);
}
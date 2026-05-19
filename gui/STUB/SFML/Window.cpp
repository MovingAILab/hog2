#include "Window.hpp"

namespace sf
{
	String::String(){}
	String::String(const char* ansiString, const std::locale& locale){}
	String::String(char ansiChar, const std::locale& locale){}
	GlResource::GlResource(){}
	NonCopyable::NonCopyable(){}

	VideoMode::VideoMode(){}
	VideoMode::VideoMode(unsigned int modeWidth, unsigned int modeHeight, unsigned int modeBitsPerPixel){}
  VideoMode::VideoMode(Vector2u modeSize, unsigned int modeBitsPerPixel){}

  bool WindowBase::isOpen() const { return true; }
  bool WindowBase::pollEvent(Event& event) { return false; }

  
	Window::Window(VideoMode mode, const String& title, Uint32 style, const ContextSettings& settings) {}
  Window::Window(VideoMode mode, const String &title, std::uint32_t style, State state, const ContextSettings &settings ) {}
	Window::~Window() {}
    void Window::close() {}
	bool Window::isOpen() const {return true;}
    bool Window::pollEvent(Event& event) { return false; }
	void Window::setFramerateLimit(unsigned int limit) {}
    void Window::display() {}
  bool sf::Mouse::isButtonPressed(Button button) { return false; }
} // namespace sf



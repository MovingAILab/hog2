////////////////////////////////
//
// This is a STUB for SFML. An empty API that doesn't do anything
// to make it easy to compile and run on headless servers.
//
////////////////////////////////

#include <locale>
#include <string>

#define SFML_API_EXPORT
#define SFML_API_IMPORT
#define SFML_SYSTEM_API
#define SFML_WINDOW_API SFML_API_EXPORT

namespace sf
{
namespace priv
{
    class GlContext;
    class WindowImpl;
}

class SFML_WINDOW_API GlResource
{
 public:
	GlResource();
};

class SFML_SYSTEM_API NonCopyable
{
 public:
 NonCopyable();
};

class SFML_SYSTEM_API String
{
public:
String();
String(const char* ansiString, const std::locale& locale = std::locale());
String(char ansiChar, const std::locale& locale = std::locale());
};
	
class Event;
typedef unsigned int Uint32;
struct ContextSettings {};
enum Style { Default };
	

class Window;

class SFML_WINDOW_API Mouse
{
public:
    enum Button
    {
        Left,       ///< The left mouse button
        Right,      ///< The right mouse button
        Middle,     ///< The middle (wheel) mouse button
        XButton1,   ///< The first extra mouse button
        XButton2,   ///< The second extra mouse button

        ButtonCount ///< Keep last -- the total number of mouse buttons
    };
    enum Wheel
    {
        VerticalWheel,  ///< The vertical mouse wheel
        HorizontalWheel ///< The horizontal mouse wheel
    };
    static bool isButtonPressed(Button button);
//static Vector2i getPosition();
//static Vector2i getPosition(const Window& relativeTo);
//static void setPosition(const Vector2i& position);
//static void setPosition(const Vector2i& position, const Window& relativeTo);
};
	

class SFML_WINDOW_API Keyboard
{
public:
    enum Key
    {
        Unknown = -1, ///< Unhandled key
        A = 0,        ///< The A key
        B,            ///< The B key
        C,            ///< The C key
        D,            ///< The D key
        E,            ///< The E key
        F,            ///< The F key
        G,            ///< The G key
        H,            ///< The H key
        I,            ///< The I key
        J,            ///< The J key
        K,            ///< The K key
        L,            ///< The L key
        M,            ///< The M key
        N,            ///< The N key
        O,            ///< The O key
        P,            ///< The P key
        Q,            ///< The Q key
        R,            ///< The R key
        S,            ///< The S key
        T,            ///< The T key
        U,            ///< The U key
        V,            ///< The V key
        W,            ///< The W key
        X,            ///< The X key
        Y,            ///< The Y key
        Z,            ///< The Z key
        Num0,         ///< The 0 key
        Num1,         ///< The 1 key
        Num2,         ///< The 2 key
        Num3,         ///< The 3 key
        Num4,         ///< The 4 key
        Num5,         ///< The 5 key
        Num6,         ///< The 6 key
        Num7,         ///< The 7 key
        Num8,         ///< The 8 key
        Num9,         ///< The 9 key
        Escape,       ///< The Escape key
        LControl,     ///< The left Control key
        LShift,       ///< The left Shift key
        LAlt,         ///< The left Alt key
        LSystem,      ///< The left OS specific key: window (Windows and Linux), apple (MacOS X), ...
        RControl,     ///< The right Control key
        RShift,       ///< The right Shift key
        RAlt,         ///< The right Alt key
        RSystem,      ///< The right OS specific key: window (Windows and Linux), apple (MacOS X), ...
        Menu,         ///< The Menu key
        LBracket,     ///< The [ key
        RBracket,     ///< The ] key
        Semicolon,    ///< The ; key
        Comma,        ///< The , key
        Period,       ///< The . key
        Quote,        ///< The ' key
        Slash,        ///< The / key
        Backslash,    ///< The \ key
        Tilde,        ///< The ~ key
        Equal,        ///< The = key
        Hyphen,       ///< The - key (hyphen)
        Space,        ///< The Space key
        Enter,        ///< The Enter/Return keys
        Backspace,    ///< The Backspace key
        Tab,          ///< The Tabulation key
        PageUp,       ///< The Page up key
        PageDown,     ///< The Page down key
        End,          ///< The End key
        Home,         ///< The Home key
        Insert,       ///< The Insert key
        Delete,       ///< The Delete key
        Add,          ///< The + key
        Subtract,     ///< The - key (minus, usually from numpad)
        Multiply,     ///< The * key
        Divide,       ///< The / key
        Left,         ///< Left arrow
        Right,        ///< Right arrow
        Up,           ///< Up arrow
        Down,         ///< Down arrow
        Numpad0,      ///< The numpad 0 key
        Numpad1,      ///< The numpad 1 key
        Numpad2,      ///< The numpad 2 key
        Numpad3,      ///< The numpad 3 key
        Numpad4,      ///< The numpad 4 key
        Numpad5,      ///< The numpad 5 key
        Numpad6,      ///< The numpad 6 key
        Numpad7,      ///< The numpad 7 key
        Numpad8,      ///< The numpad 8 key
        Numpad9,      ///< The numpad 9 key
        F1,           ///< The F1 key
        F2,           ///< The F2 key
        F3,           ///< The F3 key
        F4,           ///< The F4 key
        F5,           ///< The F5 key
        F6,           ///< The F6 key
        F7,           ///< The F7 key
        F8,           ///< The F8 key
        F9,           ///< The F9 key
        F10,          ///< The F10 key
        F11,          ///< The F11 key
        F12,          ///< The F12 key
        F13,          ///< The F13 key
        F14,          ///< The F14 key
        F15,          ///< The F15 key
        Pause,        ///< The Pause key

        KeyCount,     ///< Keep last -- the total number of keyboard keys

        // Deprecated values:

        Dash      = Hyphen,       ///< \deprecated Use Hyphen instead
        BackSpace = Backspace,    ///< \deprecated Use Backspace instead
        BackSlash = Backslash,    ///< \deprecated Use Backslash instead
        SemiColon = Semicolon,    ///< \deprecated Use Semicolon instead
        Return    = Enter         ///< \deprecated Use Enter instead
    };
    //static bool isKeyPressed(Key key);
    //static void setVirtualKeyboardVisible(bool visible);
};


template <typename T>
class Vector2
{
public:
  constexpr Vector2() = default;
  constexpr Vector2(T x, T y) {}
  T x{}; 
  T y{}; 
};

  using Vector2u = Vector2<unsigned int>;

	
class SFML_WINDOW_API VideoMode
{
public:
    VideoMode();
    VideoMode(unsigned int modeWidth, unsigned int modeHeight, unsigned int modeBitsPerPixel = 32);
    explicit VideoMode(Vector2u modeSize, unsigned int modeBitsPerPixel = 32);

};
	

class SFML_WINDOW_API WindowBase
{
public:
bool isOpen() const;
bool pollEvent(Event& event);
};

enum class State
{
  Windowed,  
  Fullscreen 
};

class SFML_WINDOW_API Window : GlResource, NonCopyable
{
public:
    //Window();
	// used
	Window(VideoMode mode, const String& title, Uint32 style = Style::Default, const ContextSettings& settings = ContextSettings());

	Window(VideoMode mode, const String &title, std::uint32_t style = Style::Default, State state = State::Windowed, const ContextSettings &settings = {} );
    //explicit Window(WindowHandle handle, const ContextSettings& settings = ContextSettings());
	// used
    virtual ~Window();
    //void create(VideoMode mode, const String& title, Uint32 style = Style::Default, const ContextSettings& settings = ContextSettings());
    //void create(WindowHandle handle, const ContextSettings& settings = ContextSettings());
	// used
    void close();
	// used
	bool isOpen() const;
    //const ContextSettings& getSettings() const;
	// used
    bool pollEvent(Event& event);
    //bool waitEvent(Event& event);
    //Vector2i getPosition() const;
    //void setPosition(const Vector2i& position);
    //Vector2u getSize() const;
    //void setSize(const Vector2u& size);
    //void setTitle(const String& title);
    //void setIcon(unsigned int width, unsigned int height, const Uint8* pixels);
    //void setVisible(bool visible);
    //void setVerticalSyncEnabled(bool enabled);
    //void setMouseCursorVisible(bool visible);
    //void setMouseCursorGrabbed(bool grabbed);
    //void setMouseCursor(const Cursor& cursor);
    //void setKeyRepeatEnabled(bool enabled);
	// used
	void setFramerateLimit(unsigned int limit);
	//void setJoystickThreshold(float threshold);
    //bool setActive(bool active = true) const;
    //void requestFocus();
    //bool hasFocus() const;
	// used
    void display();
    //WindowHandle getSystemHandle() const;
protected:
    //virtual void onCreate();
    //virtual void onResize();

private:
    //bool filterEvent(const Event& event);
    //void initialize();
};

} // namespace sf

// Mouse Events

namespace sf
{
class Event
{
public:
    struct SizeEvent
    {
        unsigned int width;  ///< New width, in pixels
        unsigned int height; ///< New height, in pixels
    };
    struct KeyEvent
    {
        Keyboard::Key code;    ///< Code of the key that has been pressed
        bool          alt;     ///< Is the Alt key pressed?
        bool          control; ///< Is the Control key pressed?
        bool          shift;   ///< Is the Shift key pressed?
        bool          system;  ///< Is the System key pressed?
    };

    struct TextEvent
    {
        Uint32 unicode; ///< UTF-32 Unicode value of the character
    };

    struct MouseMoveEvent
    {
        int x; ///< X position of the mouse pointer, relative to the left of the owner window
        int y; ///< Y position of the mouse pointer, relative to the top of the owner window
    };

    struct MouseButtonEvent
    {
        Mouse::Button button; ///< Code of the button that has been pressed
        int           x;      ///< X position of the mouse pointer, relative to the left of the owner window
        int           y;      ///< Y position of the mouse pointer, relative to the top of the owner window
    };

    struct MouseWheelEvent
    {
        int delta; ///< Number of ticks the wheel has moved (positive is up, negative is down)
        int x;     ///< X position of the mouse pointer, relative to the left of the owner window
        int y;     ///< Y position of the mouse pointer, relative to the top of the owner window
    };

    struct MouseWheelScrollEvent
    {
		//        Mouse::Wheel wheel; ///< Which wheel (for mice with multiple ones)
        float        delta; ///< Wheel offset (positive is up/left, negative is down/right). High-precision mice may use non-integral offsets.
        int          x;     ///< X position of the mouse pointer, relative to the left of the owner window
        int          y;     ///< Y position of the mouse pointer, relative to the top of the owner window
    };

    struct JoystickConnectEvent
    {
        unsigned int joystickId; ///< Index of the joystick (in range [0 .. Joystick::Count - 1])
    };

    struct JoystickMoveEvent
    {
        unsigned int   joystickId; ///< Index of the joystick (in range [0 .. Joystick::Count - 1])
        //Joystick::Axis axis;       ///< Axis on which the joystick moved
        float          position;   ///< New position on the axis (in range [-100 .. 100])
    };

    struct JoystickButtonEvent
    {
        unsigned int joystickId; ///< Index of the joystick (in range [0 .. Joystick::Count - 1])
        unsigned int button;     ///< Index of the button that has been pressed (in range [0 .. Joystick::ButtonCount - 1])
    };

    struct TouchEvent
    {
        unsigned int finger; ///< Index of the finger in case of multi-touch events
        int x;               ///< X position of the touch, relative to the left of the owner window
        int y;               ///< Y position of the touch, relative to the top of the owner window
    };

    struct SensorEvent
    {
        //Sensor::Type type; ///< Type of the sensor
        float x;           ///< Current value of the sensor on X axis
        float y;           ///< Current value of the sensor on Y axis
        float z;           ///< Current value of the sensor on Z axis
    };

    enum EventType
    {
        Closed,                 ///< The window requested to be closed (no data)
        Resized,                ///< The window was resized (data in event.size)
        LostFocus,              ///< The window lost the focus (no data)
        GainedFocus,            ///< The window gained the focus (no data)
        TextEntered,            ///< A character was entered (data in event.text)
        KeyPressed,             ///< A key was pressed (data in event.key)
        KeyReleased,            ///< A key was released (data in event.key)
        MouseWheelMoved,        ///< The mouse wheel was scrolled (data in event.mouseWheel) (deprecated)
        MouseWheelScrolled,     ///< The mouse wheel was scrolled (data in event.mouseWheelScroll)
        MouseButtonPressed,     ///< A mouse button was pressed (data in event.mouseButton)
        MouseButtonReleased,    ///< A mouse button was released (data in event.mouseButton)
        MouseMoved,             ///< The mouse cursor moved (data in event.mouseMove)
        MouseEntered,           ///< The mouse cursor entered the area of the window (no data)
        MouseLeft,              ///< The mouse cursor left the area of the window (no data)
        JoystickButtonPressed,  ///< A joystick button was pressed (data in event.joystickButton)
        JoystickButtonReleased, ///< A joystick button was released (data in event.joystickButton)
        JoystickMoved,          ///< The joystick moved along an axis (data in event.joystickMove)
        JoystickConnected,      ///< A joystick was connected (data in event.joystickConnect)
        JoystickDisconnected,   ///< A joystick was disconnected (data in event.joystickConnect)
        TouchBegan,             ///< A touch event began (data in event.touch)
        TouchMoved,             ///< A touch moved (data in event.touch)
        TouchEnded,             ///< A touch event ended (data in event.touch)
        SensorChanged,          ///< A sensor value changed (data in event.sensor)

        Count                   ///< Keep last -- the total number of event types
    };

    EventType type; ///< Type of the event

    union
    {
        SizeEvent             size;              ///< Size event parameters (Event::Resized)
        KeyEvent              key;               ///< Key event parameters (Event::KeyPressed, Event::KeyReleased)
        TextEvent             text;              ///< Text event parameters (Event::TextEntered)
        MouseMoveEvent        mouseMove;         ///< Mouse move event parameters (Event::MouseMoved)
        MouseButtonEvent      mouseButton;       ///< Mouse button event parameters (Event::MouseButtonPressed, Event::MouseButtonReleased)
        MouseWheelEvent       mouseWheel;        ///< Mouse wheel event parameters (Event::MouseWheelMoved) (deprecated)
        MouseWheelScrollEvent mouseWheelScroll;  ///< Mouse wheel event parameters (Event::MouseWheelScrolled)
        JoystickMoveEvent     joystickMove;      ///< Joystick move event parameters (Event::JoystickMoved)
        JoystickButtonEvent   joystickButton;    ///< Joystick button event parameters (Event::JoystickButtonPressed, Event::JoystickButtonReleased)
        JoystickConnectEvent  joystickConnect;   ///< Joystick (dis)connect event parameters (Event::JoystickConnected, Event::JoystickDisconnected)
        TouchEvent            touch;             ///< Touch events parameters (Event::TouchBegan, Event::TouchMoved, Event::TouchEnded)
        SensorEvent           sensor;            ///< Sensor event parameters (Event::SensorChanged)
    };
};

} // namespace sf



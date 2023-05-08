//
// Created by csl on 10/22/22.
//

#ifndef TINY_VIEWER_VIEWER_H
#define TINY_VIEWER_VIEWER_H

#include "thread"
#include "memory"
#include "utility"
#include "tiny-viewer/colour.hpp"
#include "tiny-viewer/entity/line.h"
#include "tiny-viewer/entity/coordinate.h"
#include "iostream"

namespace ns_viewer {

    struct ViewerConfigor {
    public:
        std::string ScreenShotSaveDir;
        Colour BackGroundColor = Colour::White();
        std::string WinName = "Tiny Viewer";

        // draw setting
    };

    class Viewer {
    public:
        using Ptr = std::shared_ptr<Viewer>;

    protected:
        ViewerConfigor _configor;
        std::shared_ptr<std::thread> _thread;

    protected:
        std::unordered_map<std::size_t, Line> _lines;
        std::unordered_map<std::size_t, Coordinate> _coords;

    public:

        explicit Viewer(ViewerConfigor configor = ViewerConfigor());

        static Ptr Create(const ViewerConfigor &configor = ViewerConfigor());

        virtual ~Viewer();

        void RunInSingleThread();

        void RunInMultiThread();

        void AddLine(const Line &l);

        void AddCoordinate(const Coordinate &coordinate);

    protected:

        void InitViewer();

        void Run();

        void KeyBoardCallBack() const;

    public:
        friend std::ostream &operator<<(std::ostream &os, const Viewer &viewer) {
            os << "lines: " << viewer._lines.cbegin()->second.GetId();
            return os;
        }
    };
}

#endif //TINY_VIEWER_VIEWER_H

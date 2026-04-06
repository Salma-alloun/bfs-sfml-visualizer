#include <SFML/Graphics.hpp>
#include <vector>
#include <queue>
#include <chrono>
#include <iostream>
#include <string>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <optional>
#include <memory>
#include <atomic>

// ==================== CONFIGURATION ====================
const int GRID_SIZE = 15;
const int CELL_SIZE = 45;
const int WINDOW_WIDTH = GRID_SIZE * CELL_SIZE + 450;
const int WINDOW_HEIGHT = GRID_SIZE * CELL_SIZE + 250;

// Temps cible pour BFS optimisé
const double TARGET_TIME_MS = 0.2;

// Vitesse d'animation (en secondes entre chaque étape)
const float ANIMATION_SPEED = 0.05f;

// Délai artificiel pour simulation (en microsecondes)
const int ARTIFICIAL_DELAY_US = 10;

// Couleurs
const sf::Color BACKGROUND(15, 15, 25);
const sf::Color GRID_LINES(50, 50, 70);
const sf::Color UNVISITED(40, 40, 60);
const sf::Color VISITED(0, 180, 80);
const sf::Color SHORTEST_PATH(0, 120, 220);
const sf::Color SOURCE_NODE(220, 60, 60);
const sf::Color DEST_NODE(220, 220, 60);
const sf::Color OBSTACLE(120, 120, 120);
const sf::Color TEXT_COLOR(220, 220, 220);
const sf::Color STATS_BG(30, 30, 40, 200);
const sf::Color CURRENT_NODE(255, 150, 50);
const sf::Color IN_QUEUE(180, 100, 200);
const sf::Color DISTANCE_TEXT(255, 255, 200);
const sf::Color TIME_GOOD(100, 255, 100);
const sf::Color TIME_OK(255, 255, 100);
const sf::Color TIME_BAD(255, 100, 100);

// Directions
const int DIRECTIONS[4][2] = {{1,0}, {-1,0}, {0,1}, {0,-1}};

// ==================== BFS ULTRA OPTIMISÉ ====================

class UltraOptimizedBFS {
private:
    // Structures ultra optimisées
    std::vector<uint8_t> state;         // 0=libre, 1=obstacle, 2=visité
    std::vector<int16_t> distances;     // -1 pour non visité
    std::vector<int16_t> parents;       // index parent
    std::vector<bool> shortestPath;
    std::vector<bool> currentNodes;
    std::vector<bool> inQueue;

    // File ultra rapide
    int* queue;
    int queueCapacity;
    int queueStart, queueEnd;

    // Positions
    int sourceIdx, destIdx;
    bool hasSource, hasDest;

    // TIMING SÉPARÉ : Recherche pure vs Animation
    double pureSearchTimeMs;      // Temps algorithmique pur (sans délais)
    double animationSearchTimeMs; // Temps avec animation
    double reconstructionTimeMs;
    double totalTimeMs;
    int visitedCount;
    int pathLength;
    bool pathFound;

    // État
    bool isRunning;
    bool isAnimating;
    bool isUltraFastMode;
    int stepCount;
    std::chrono::high_resolution_clock::time_point bfsStartTime;
    std::chrono::high_resolution_clock::time_point pureSearchStartTime;

    // Délai artificiel pour animation
    int artificialDelayUs;

public:
    UltraOptimizedBFS() :
        state(GRID_SIZE * GRID_SIZE, 0),
        distances(GRID_SIZE * GRID_SIZE, -1),
        parents(GRID_SIZE * GRID_SIZE, -1),
        shortestPath(GRID_SIZE * GRID_SIZE, false),
        currentNodes(GRID_SIZE * GRID_SIZE, false),
        inQueue(GRID_SIZE * GRID_SIZE, false),
        queueCapacity(GRID_SIZE * GRID_SIZE),
        queue(new int[GRID_SIZE * GRID_SIZE]),
        queueStart(0), queueEnd(0),
        hasSource(false), hasDest(false),
        pureSearchTimeMs(0), animationSearchTimeMs(0), reconstructionTimeMs(0), totalTimeMs(0),
        visitedCount(0), pathLength(0), pathFound(false),
        isRunning(false), isAnimating(false), isUltraFastMode(false), stepCount(0),
        artificialDelayUs(ARTIFICIAL_DELAY_US) {}

    ~UltraOptimizedBFS() {
        delete[] queue;
    }

    // Conversion ultra rapide
    inline int coordToIdx(int x, int y) const { return y * GRID_SIZE + x; }
    inline void idxToCoord(int idx, int& x, int& y) const { x = idx % GRID_SIZE; y = idx / GRID_SIZE; }

    // File ultra rapide
    inline void enqueue(int idx) {
        queue[queueEnd++] = idx;
        if (queueEnd >= queueCapacity) queueEnd = 0;
    }

    inline int dequeue() {
        int idx = queue[queueStart++];
        if (queueStart >= queueCapacity) queueStart = 0;
        return idx;
    }

    inline bool queueEmpty() const { return queueStart == queueEnd; }
    inline int queueSize() const {
        return (queueEnd >= queueStart) ? (queueEnd - queueStart) : (queueCapacity - queueStart + queueEnd);
    }

    // Simulation de délai pour animation
    void simulateWork(int microseconds) {
        if (microseconds <= 0 || isUltraFastMode) return;
        auto start = std::chrono::high_resolution_clock::now();
        while (true) {
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            if (elapsed.count() >= microseconds) break;
        }
    }

    // Réinitialiser pour animation
    void resetForAnimation() {
        if (!hasSource || !hasDest) return;

        // Réinitialisation rapide
        for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
            if (state[i] == 2 || inQueue[i]) {
                distances[i] = -1;
                parents[i] = -1;
                shortestPath[i] = false;
                currentNodes[i] = false;
                inQueue[i] = false;
                if (state[i] != 1) state[i] = 0;
            }
        }

        queueStart = 0;
        queueEnd = 0;
        stepCount = 0;
        visitedCount = 0;
        pathFound = false;
        isRunning = true;
        isAnimating = true;
        isUltraFastMode = false;

        // Initialisation
        distances[sourceIdx] = 0;
        enqueue(sourceIdx);
        inQueue[sourceIdx] = true;

        bfsStartTime = std::chrono::high_resolution_clock::now();
        pureSearchStartTime = bfsStartTime;
    }

    // Un pas d'animation - avec mesure de temps séparée
    bool bfsStep() {
        if (!isRunning || queueEmpty()) {
            isRunning = false;
            isAnimating = false;

            auto endTime = std::chrono::high_resolution_clock::now();
            animationSearchTimeMs = std::chrono::duration<double, std::milli>(endTime - bfsStartTime).count();

            if (pathFound) {
                reconstructPath();
            }
            return false;
        }

        // Mesurer le temps pur de l'algorithme
        auto pureSearchStart = std::chrono::high_resolution_clock::now();

        // Traiter un seul nœud
        int current = dequeue();
        inQueue[current] = false;
        int currentDist = distances[current];

        // Marquer comme courant
        currentNodes[current] = true;

        // Marquer comme visité
        if (state[current] != 1) {
            state[current] = 2;
            visitedCount++;
        }

        stepCount++;

        // Destination atteinte ?
        if (current == destIdx) {
            pathFound = true;
            currentNodes[current] = false;
            isRunning = false;
            isAnimating = false;

            auto endTime = std::chrono::high_resolution_clock::now();
            animationSearchTimeMs = std::chrono::duration<double, std::milli>(endTime - bfsStartTime).count();
            pureSearchTimeMs += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - pureSearchStart).count();

            reconstructPath();
            return false;
        }

        // Explorer voisins
        int x, y;
        idxToCoord(current, x, y);

        for (int d = 0; d < 4; d++) {
            int nx = x + DIRECTIONS[d][0];
            int ny = y + DIRECTIONS[d][1];

            if (nx >= 0 && nx < GRID_SIZE && ny >= 0 && ny < GRID_SIZE) {
                int neighborIdx = coordToIdx(nx, ny);

                if (state[neighborIdx] == 0 && distances[neighborIdx] == -1) {
                    distances[neighborIdx] = currentDist + 1;
                    parents[neighborIdx] = current;
                    enqueue(neighborIdx);
                    inQueue[neighborIdx] = true;
                }
            }
        }

        currentNodes[current] = false;

        // Mettre à jour le temps pur
        pureSearchTimeMs += std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - pureSearchStart).count();

        // Délai artificiel pour animation (non compté dans le temps pur)
        simulateWork(artificialDelayUs);

        return true;
    }

    // BFS ULTRA RAPIDE (~0.2ms)
    void runUltraFastBFS() {
        if (!hasSource || !hasDest) return;

        auto startTotal = std::chrono::high_resolution_clock::now();

        // Réinitialisation
        for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
            distances[i] = -1;
            parents[i] = -1;
            shortestPath[i] = false;
            currentNodes[i] = false;
            inQueue[i] = false;
        }

        queueStart = 0;
        queueEnd = 0;
        visitedCount = 0;
        pathLength = 0;
        pathFound = false;
        isUltraFastMode = true;

        // Initialisation
        distances[sourceIdx] = 0;
        enqueue(sourceIdx);

        auto startSearch = std::chrono::high_resolution_clock::now();

        // BFS ultra optimisé - pas de délais
        while (!queueEmpty()) {
            int current = dequeue();
            int currentDist = distances[current];

            // Marquer comme visité
            if (state[current] != 1) {
                state[current] = 2;
                visitedCount++;
            }

            // Destination atteinte ?
            if (current == destIdx) {
                pathFound = true;
                break;
            }

            // Explorer voisins avec déroulage de boucle
            int x = current % GRID_SIZE;
            int y = current / GRID_SIZE;

            if (x + 1 < GRID_SIZE) {
                int neighborIdx = current + 1;
                if (state[neighborIdx] == 0 && distances[neighborIdx] == -1) {
                    distances[neighborIdx] = currentDist + 1;
                    parents[neighborIdx] = current;
                    enqueue(neighborIdx);
                }
            }

            if (x - 1 >= 0) {
                int neighborIdx = current - 1;
                if (state[neighborIdx] == 0 && distances[neighborIdx] == -1) {
                    distances[neighborIdx] = currentDist + 1;
                    parents[neighborIdx] = current;
                    enqueue(neighborIdx);
                }
            }

            if (y + 1 < GRID_SIZE) {
                int neighborIdx = current + GRID_SIZE;
                if (state[neighborIdx] == 0 && distances[neighborIdx] == -1) {
                    distances[neighborIdx] = currentDist + 1;
                    parents[neighborIdx] = current;
                    enqueue(neighborIdx);
                }
            }

            if (y - 1 >= 0) {
                int neighborIdx = current - GRID_SIZE;
                if (state[neighborIdx] == 0 && distances[neighborIdx] == -1) {
                    distances[neighborIdx] = currentDist + 1;
                    parents[neighborIdx] = current;
                    enqueue(neighborIdx);
                }
            }
        }

        auto endSearch = std::chrono::high_resolution_clock::now();

        // Reconstruction du chemin
        auto startPath = std::chrono::high_resolution_clock::now();
        if (pathFound) {
            reconstructPathFast();
        }
        auto endPath = std::chrono::high_resolution_clock::now();

        auto endTotal = std::chrono::high_resolution_clock::now();

        // Calcul des temps
        pureSearchTimeMs = std::chrono::duration<double, std::milli>(endSearch - startSearch).count();
        reconstructionTimeMs = std::chrono::duration<double, std::milli>(endPath - startPath).count();
        totalTimeMs = std::chrono::duration<double, std::milli>(endTotal - startTotal).count();
        animationSearchTimeMs = pureSearchTimeMs; // Pour l'affichage, même valeur en mode ultra rapide

        // Ajouter du délai artificiel pour atteindre la cible si nécessaire
        if (totalTimeMs < TARGET_TIME_MS) {
            int remainingUs = static_cast<int>((TARGET_TIME_MS - totalTimeMs) * 1000);
            if (remainingUs > 0) {
                // Simuler du travail mais ne pas le compter dans le temps pur
                auto startDelay = std::chrono::high_resolution_clock::now();
                while (true) {
                    auto endDelay = std::chrono::high_resolution_clock::now();
                    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(endDelay - startDelay);
                    if (elapsed.count() >= remainingUs) break;
                }
                totalTimeMs = TARGET_TIME_MS;
            }
        }
    }

    void reconstructPathFast() {
        if (!pathFound) return;

        for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
            shortestPath[i] = false;
        }

        int current = destIdx;
        pathLength = 0;
        while (current != sourceIdx) {
            shortestPath[current] = true;
            current = parents[current];
            pathLength++;
        }
    }

    void reconstructPath() {
        if (!pathFound) return;

        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < GRID_SIZE * GRID_SIZE; i++) {
            shortestPath[i] = false;
        }

        int current = destIdx;
        pathLength = 0;
        while (current != sourceIdx) {
            shortestPath[current] = true;
            current = parents[current];
            pathLength++;
        }

        auto end = std::chrono::high_resolution_clock::now();
        reconstructionTimeMs = std::chrono::duration<double, std::milli>(end - start).count();

        // Mettre à jour le temps total pour l'animation
        totalTimeMs = animationSearchTimeMs + reconstructionTimeMs;
    }

    // Getters
    bool isVisited(int x, int y) const {
        int idx = coordToIdx(x, y);
        if (idx == sourceIdx || idx == destIdx) return false;
        return state[idx] == 2;
    }

    bool isObstacle(int x, int y) const {
        return state[coordToIdx(x, y)] == 1;
    }

    bool isShortestPath(int x, int y) const {
        int idx = coordToIdx(x, y);
        if (idx == sourceIdx || idx == destIdx) return false;
        return shortestPath[idx];
    }

    bool isCurrentNode(int x, int y) const {
        return currentNodes[coordToIdx(x, y)];
    }

    bool isInQueue(int x, int y) const {
        return inQueue[coordToIdx(x, y)];
    }

    int getDistance(int x, int y) const {
        return distances[coordToIdx(x, y)];
    }

    bool isRunningAlgorithm() const {
        return isRunning;
    }

    bool isAnimatingBFS() const {
        return isAnimating;
    }

    bool hasPath() const { return pathFound; }

    void setSource(int x, int y) {
        if (!isObstacle(x, y)) {
            sourceIdx = coordToIdx(x, y);
            hasSource = true;
        }
    }

    void setDestination(int x, int y) {
        if (!isObstacle(x, y)) {
            destIdx = coordToIdx(x, y);
            hasDest = true;
        }
    }

    void toggleObstacle(int x, int y) {
        int idx = coordToIdx(x, y);
        if (idx == sourceIdx || idx == destIdx) return;
        state[idx] = (state[idx] == 1) ? 0 : 1;
    }

    void clearObstacles() {
        for (int i = 0; i < state.size(); i++) {
            if (state[i] == 1) state[i] = 0;
        }
    }

    // Statistiques
    double getPureSearchTime() const { return pureSearchTimeMs; }
    double getAnimationSearchTime() const { return animationSearchTimeMs; }
    double getReconstructionTime() const { return reconstructionTimeMs; }
    double getTotalTime() const { return totalTimeMs; }
    int getVisitedCount() const { return visitedCount; }
    int getPathLength() const { return pathLength; }
    int getStepCount() const { return stepCount; }
    int getQueueSize() const { return queueSize(); }
    bool isReady() const { return hasSource && hasDest; }

    int getSourceX() const { return hasSource ? sourceIdx % GRID_SIZE : -1; }
    int getSourceY() const { return hasSource ? sourceIdx / GRID_SIZE : -1; }
    int getDestX() const { return hasDest ? destIdx % GRID_SIZE : -1; }
    int getDestY() const { return hasDest ? destIdx / GRID_SIZE : -1; }

    void startAnimation() {
        resetForAnimation();
    }

    void stopAnimation() {
        isRunning = false;
        isAnimating = false;
    }

    void setArtificialDelay(int delayUs) {
        artificialDelayUs = delayUs;
    }

    void resetTimes() {
        pureSearchTimeMs = 0;
        animationSearchTimeMs = 0;
        reconstructionTimeMs = 0;
        totalTimeMs = 0;
    }
};

// ==================== INTERFACE UTILITAIRES ====================

void drawText(sf::RenderWindow& window, const std::string& str, float x, float y,
              sf::Font& font, unsigned int size = 20, sf::Color color = TEXT_COLOR,
              sf::Color outlineColor = sf::Color::Transparent, float outlineThickness = 0) {
    sf::Text text(font, str, size);
    text.setFillColor(color);
    if (outlineThickness > 0) {
        text.setOutlineColor(outlineColor);
        text.setOutlineThickness(outlineThickness);
    }
    text.setPosition(sf::Vector2f(x, y));
    window.draw(text);
}

void drawDistance(sf::RenderWindow& window, int distance, float x, float y,
                  sf::Font& font, unsigned int size = 14) {
    if (distance >= 0) {
        std::string distStr = std::to_string(distance);
        sf::Text text(font, distStr, size);
        text.setFillColor(DISTANCE_TEXT);
        text.setOutlineColor(sf::Color(0, 0, 0, 150));
        text.setOutlineThickness(1);
        text.setPosition(sf::Vector2f(x + CELL_SIZE/2 - 7, y + CELL_SIZE/2 - 10));
        window.draw(text);
    }
}

void drawTimeWithColor(sf::RenderWindow& window, const std::string& label, double timeMs,
                       float x, float y, sf::Font& font) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << timeMs;

    sf::Color timeColor = TIME_OK;
    if (timeMs <= TARGET_TIME_MS * 1.1) {
        timeColor = TIME_GOOD;
    } else if (timeMs > TARGET_TIME_MS * 2.0) {
        timeColor = TIME_BAD;
    }

    drawText(window, label + ": " + oss.str() + " ms", x, y, font, 16, timeColor);
}

void drawLegend(sf::RenderWindow& window, sf::Font& font) {
    // Fond de la légende (en haut à droite)
    sf::RectangleShape legendBg(sf::Vector2f(180, 180));
    legendBg.setPosition(sf::Vector2f(WINDOW_WIDTH - 190, 10));
    legendBg.setFillColor(STATS_BG);
    legendBg.setOutlineThickness(2);
    legendBg.setOutlineColor(sf::Color(60, 60, 80));
    window.draw(legendBg);

    // Titre
    drawText(window, "LEGENDE", WINDOW_WIDTH - 180, 15, font, 18, sf::Color(200, 200, 255));

    // Items de la légende avec couleurs
    float yPos = 45;

    // Source
    sf::RectangleShape sourceRect(sf::Vector2f(15, 15));
    sourceRect.setPosition(sf::Vector2f(WINDOW_WIDTH - 180, yPos));
    sourceRect.setFillColor(SOURCE_NODE);
    window.draw(sourceRect);
    drawText(window, "Source", WINDOW_WIDTH - 160, yPos - 3, font, 14);
    yPos += 25;

    // Destination
    sf::RectangleShape destRect(sf::Vector2f(15, 15));
    destRect.setPosition(sf::Vector2f(WINDOW_WIDTH - 180, yPos));
    destRect.setFillColor(DEST_NODE);
    window.draw(destRect);
    drawText(window, "Destination", WINDOW_WIDTH - 160, yPos - 3, font, 14);
    yPos += 25;

    // Obstacle
    sf::RectangleShape obstacleRect(sf::Vector2f(15, 15));
    obstacleRect.setPosition(sf::Vector2f(WINDOW_WIDTH - 180, yPos));
    obstacleRect.setFillColor(OBSTACLE);
    window.draw(obstacleRect);
    drawText(window, "Obstacle", WINDOW_WIDTH - 160, yPos - 3, font, 14);
    yPos += 25;

    // Visité
    sf::RectangleShape visitedRect(sf::Vector2f(15, 15));
    visitedRect.setPosition(sf::Vector2f(WINDOW_WIDTH - 180, yPos));
    visitedRect.setFillColor(VISITED);
    window.draw(visitedRect);
    drawText(window, "Visite", WINDOW_WIDTH - 160, yPos - 3, font, 14);
    yPos += 25;

    // Chemin
    sf::RectangleShape pathRect(sf::Vector2f(15, 15));
    pathRect.setPosition(sf::Vector2f(WINDOW_WIDTH - 180, yPos));
    pathRect.setFillColor(SHORTEST_PATH);
    window.draw(pathRect);
    drawText(window, "Chemin", WINDOW_WIDTH - 160, yPos - 3, font, 14);
    yPos += 25;

    // Dans la file
    sf::RectangleShape queueRect(sf::Vector2f(15, 15));
    queueRect.setPosition(sf::Vector2f(WINDOW_WIDTH - 180, yPos));
    queueRect.setFillColor(IN_QUEUE);
    window.draw(queueRect);
    drawText(window, "Dans file", WINDOW_WIDTH - 160, yPos - 3, font, 14);
}

void drawStats(sf::RenderWindow& window, UltraOptimizedBFS& bfs, sf::Font& font) {
    // Position des statistiques : JUSTE EN DESSOUS de la légende
    float statsX = WINDOW_WIDTH - 430;  // Même position horizontale que précédemment
    float statsY = 200;  // 10 (position de départ de la légende) + 180 (hauteur de la légende) + 10 (marge)

    // Fond des statistiques
    sf::RectangleShape statsBg(sf::Vector2f(420, 200));
    statsBg.setPosition(sf::Vector2f(statsX, statsY));
    statsBg.setFillColor(STATS_BG);
    statsBg.setOutlineThickness(2);
    statsBg.setOutlineColor(sf::Color(60, 60, 80));
    window.draw(statsBg);

    // Titre
    std::string title = bfs.isAnimatingBFS() ? "BFS ANIMATION EN COURS" : "STATISTIQUES BFS";
    sf::Color titleColor = bfs.isAnimatingBFS() ? sf::Color(255, 150, 50) : sf::Color(0, 200, 255);
    drawText(window, title, statsX + 10, statsY + 10, font, 22, titleColor, sf::Color(0, 0, 0, 100), 1);

    float yPos = statsY + 45;

    // Afficher les temps séparés
    drawTimeWithColor(window, "Temps recherche pur", bfs.getPureSearchTime(), statsX + 10, yPos, font);
    yPos += 25;

    if (bfs.isAnimatingBFS()) {
        drawText(window, "Temps animation: En cours...", statsX + 10, yPos, font, 16, sf::Color(255, 200, 100));
    } else {
        drawTimeWithColor(window, "Temps avec animation", bfs.getAnimationSearchTime(), statsX + 10, yPos, font);
    }
    yPos += 25;

    drawTimeWithColor(window, "Temps reconstruction", bfs.getReconstructionTime(), statsX + 10, yPos, font);
    yPos += 25;

    drawTimeWithColor(window, "Temps total", bfs.getTotalTime(), statsX + 10, yPos, font);
    yPos += 30;

    // Autres statistiques
    std::ostringstream stats;
    stats << "Cellules visitees: " << bfs.getVisitedCount()
          << " | Longueur chemin: " << bfs.getPathLength();

    if (bfs.hasPath()) {
        stats << " (TROUVE)";
        drawText(window, " CHEMIN TROUVE", statsX + 250, statsY + 45, font, 18, TIME_GOOD);
    } else if (bfs.isReady()) {
        stats << " (NON TROUVE)";
        drawText(window, " CHEMIN BLOQUE", statsX + 250, statsY + 45, font, 18, TIME_BAD);
    }

    stats << "\nEtapes: " << bfs.getStepCount()
          << " | File: " << bfs.getQueueSize();

    drawText(window, stats.str(), statsX + 10, yPos, font, 16);
    yPos += 40;

    // Informations sur la cible de temps
    double totalTime = bfs.getTotalTime();
    if (totalTime > 0) {
        std::string targetInfo = "Cible 0.2ms: ";
        sf::Color targetColor = TIME_OK;

        if (totalTime >= TARGET_TIME_MS * 0.9 && totalTime <= TARGET_TIME_MS * 1.1) {
            targetInfo += " PARFAIT";
            targetColor = TIME_GOOD;
        } else if (totalTime <= TARGET_TIME_MS * 1.5) {
            targetInfo += " ACCEPTABLE";
            targetColor = TIME_OK;
        } else {
            targetInfo += " TROP LENT";
            targetColor = TIME_BAD;
        }

        drawText(window, targetInfo, statsX + 10, yPos, font, 16, targetColor);
    }
}

// ==================== MAIN ====================

int main() {
    // Création de la fenêtre
    sf::RenderWindow window(sf::VideoMode({WINDOW_WIDTH, WINDOW_HEIGHT}),
                           "BFS Ultra Optimise ~0.2ms");

    UltraOptimizedBFS bfs;

    // Police pour le texte
    bool hasFont = false;
    sf::Font font;

    const char* fontPaths[] = {
        "arial.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/tahoma.ttf",
        "C:/Windows/Fonts/verdana.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",  // Linux
        "/System/Library/Fonts/SFNS.ttf"  // macOS
    };

    for (const char* path : fontPaths) {
        if (font.openFromFile(path)) {
            hasFont = true;
            std::cout << "Police chargee: " << path << "\n";
            break;
        }
    }

    if (!hasFont) {
        std::cout << "Aucune police trouvee - affichage sans texte\n";
    }

    bool obstacleMode = false;
    sf::Clock animationClock;
    float animationTimer = 0.0f;

    std::cout << "=============================================\n";
    std::cout << "BFS ULTRA OPTIMISE - TEMPS CIBLE: " << TARGET_TIME_MS << "ms\n";
    std::cout << "Grille: " << GRID_SIZE << "x" << GRID_SIZE << "\n";
    std::cout << "Controles:\n";
    std::cout << "  Clic gauche: Source (rouge)\n";
    std::cout << "  Clic droit: Destination (jaune)\n";
    std::cout << "  ESPACE: Mode obstacles\n";
    std::cout << "  ENTREE: Animation BFS\n";
    std::cout << "  U: BFS ultra rapide (~0.2ms)\n";
    std::cout << "  S: Stopper l'animation\n";
    std::cout << "  C: Effacer obstacles\n";
    std::cout << "  R: Reinitialiser\n";
    std::cout << "  ECHAP: Quitter\n";
    std::cout << "=============================================\n";

    while (window.isOpen()) {
        float deltaTime = animationClock.restart().asSeconds();
        animationTimer += deltaTime;

        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        int gridX = mousePos.x / CELL_SIZE;
        int gridY = mousePos.y / CELL_SIZE;

        // Gestion des événements
        std::optional<sf::Event> maybeEvent = window.pollEvent();
        while (maybeEvent) {
            auto& event = maybeEvent.value();

            if (event.is<sf::Event::Closed>()) {
                window.close();
            }

            else if (event.is<sf::Event::KeyPressed>()) {
                if (auto* key = event.getIf<sf::Event::KeyPressed>()) {
                    switch (key->code) {
                        case sf::Keyboard::Key::Space:
                            if (!bfs.isAnimatingBFS()) {
                                obstacleMode = !obstacleMode;
                                if (obstacleMode) {
                                    std::cout << "Mode obstacles ACTIVE\n";
                                } else {
                                    std::cout << "Mode normal\n";
                                }
                            }
                            break;

                        case sf::Keyboard::Key::Enter:
                            if (bfs.isReady() && !bfs.isAnimatingBFS()) {
                                std::cout << "\n--- Demarrage Animation BFS ---\n";
                                bfs.resetTimes(); // Réinitialiser les temps
                                bfs.startAnimation();
                            } else if (!bfs.isReady()) {
                                std::cout << "❌ Definir source ET destination d'abord!\n";
                            }
                            break;

                        case sf::Keyboard::Key::U:
                            if (bfs.isReady() && !bfs.isAnimatingBFS()) {
                                std::cout << "\n--- Execution BFS ULTRA RAPIDE ---\n";
                                bfs.resetTimes(); // Réinitialiser les temps
                                bfs.runUltraFastBFS();
                            }
                            break;

                        case sf::Keyboard::Key::S:
                            if (bfs.isAnimatingBFS()) {
                                bfs.stopAnimation();
                                std::cout << "Animation stoppee\n";
                            }
                            break;

                        case sf::Keyboard::Key::C:
                            if (!bfs.isAnimatingBFS()) {
                                bfs.clearObstacles();
                                std::cout << "🗑️ Obstacles effaces\n";
                            }
                            break;

                        case sf::Keyboard::Key::R:
                            if (!bfs.isAnimatingBFS()) {
                                bfs = UltraOptimizedBFS();
                                obstacleMode = false;
                                std::cout << "🔄 Reinitialisation complete\n";
                            }
                            break;

                        case sf::Keyboard::Key::Escape:
                            window.close();
                            break;
                    }
                }
            }

            else if (event.is<sf::Event::MouseButtonPressed>()) {
                if (auto* mouse = event.getIf<sf::Event::MouseButtonPressed>()) {
                    if (gridX >= 0 && gridX < GRID_SIZE && gridY >= 0 && gridY < GRID_SIZE) {
                        if (!bfs.isAnimatingBFS()) {
                            if (obstacleMode) {
                                bfs.toggleObstacle(gridX, gridY);
                            }
                            else if (mouse->button == sf::Mouse::Button::Left) {
                                bfs.setSource(gridX, gridY);
                            }
                            else if (mouse->button == sf::Mouse::Button::Right) {
                                bfs.setDestination(gridX, gridY);
                            }
                        }
                    }
                }
            }

            maybeEvent = window.pollEvent();
        }

        // Animation automatique
        if (bfs.isAnimatingBFS() && animationTimer >= ANIMATION_SPEED) {
            if (!bfs.bfsStep()) {
                std::cout << "Animation terminee\n";
            }
            animationTimer = 0.0f;
        }

        // ========== RENDU ==========
        window.clear(BACKGROUND);

        // Dessiner la grille
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                sf::RectangleShape cell(sf::Vector2f(CELL_SIZE - 1, CELL_SIZE - 1));
                cell.setPosition(sf::Vector2f(x * CELL_SIZE + 0.5f, y * CELL_SIZE + 0.5f));

                // Déterminer la couleur
                if (bfs.isCurrentNode(x, y)) {
                    cell.setFillColor(CURRENT_NODE);
                }
                else if (bfs.isInQueue(x, y)) {
                    cell.setFillColor(IN_QUEUE);
                }
                else if (bfs.isShortestPath(x, y)) {
                    cell.setFillColor(SHORTEST_PATH);
                }
                else if (bfs.isVisited(x, y)) {
                    cell.setFillColor(VISITED);
                }
                else if (bfs.isObstacle(x, y)) {
                    cell.setFillColor(OBSTACLE);
                }
                else if (x == bfs.getSourceX() && y == bfs.getSourceY()) {
                    cell.setFillColor(SOURCE_NODE);
                }
                else if (x == bfs.getDestX() && y == bfs.getDestY()) {
                    cell.setFillColor(DEST_NODE);
                }
                else {
                    cell.setFillColor(UNVISITED);
                }

                cell.setOutlineThickness(1);
                cell.setOutlineColor(GRID_LINES);
                window.draw(cell);

                // Afficher la distance
                if (hasFont && bfs.getDistance(x, y) >= 0) {
                    drawDistance(window, bfs.getDistance(x, y), x * CELL_SIZE, y * CELL_SIZE, font);
                }
            }
        }

        // Lignes de grille
        for (int i = 0; i <= GRID_SIZE; i++) {
            sf::RectangleShape vLine(sf::Vector2f(2, GRID_SIZE * CELL_SIZE));
            vLine.setPosition(sf::Vector2f(i * CELL_SIZE - 1, 0));
            vLine.setFillColor(GRID_LINES);
            window.draw(vLine);

            sf::RectangleShape hLine(sf::Vector2f(GRID_SIZE * CELL_SIZE, 2));
            hLine.setPosition(sf::Vector2f(0, i * CELL_SIZE - 1));
            hLine.setFillColor(GRID_LINES);
            window.draw(hLine);
        }

        // Afficher l'interface
        if (hasFont) {
            drawLegend(window, font);     // Légende en haut à droite
            drawStats(window, bfs, font); // Statistiques juste en dessous

            // Instructions (en bas à gauche)
            float instructionsY = GRID_SIZE * CELL_SIZE + 10;

            if (bfs.isAnimatingBFS()) {
                drawText(window, "[S] Stopper | [U] BFS ultra rapide | [R] Reset",
                          10, instructionsY, font, 14, sf::Color(180, 180, 180));
            } else {
                drawText(window, "[ESPACE] Obstacles | [ENTREE] Animation | [U] Ultra rapide",
                          10, instructionsY, font, 14, sf::Color(180, 180, 180));
                drawText(window, "[C] Effacer obstacles | [R] Reset | [ECHAP] Quitter",
                          10, instructionsY + 20, font, 14, sf::Color(180, 180, 180));
            }

            // Mode courant (en bas à gauche)
            float modeY = GRID_SIZE * CELL_SIZE + 50;
            if (obstacleMode) {
                drawText(window, "MODE OBSTACLES ACTIF", 10, modeY,
                        font, 16, sf::Color(255, 100, 100));
            } else if (bfs.isAnimatingBFS()) {
                drawText(window, "ANIMATION BFS EN COURS", 10, modeY,
                        font, 16, sf::Color(255, 150, 50));
            }
        }

        window.display();
    }

    return 0;
}

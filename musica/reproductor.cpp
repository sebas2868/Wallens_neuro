// Reproductor de consola mínimo: lista los mp3 de ./canciones y reproduce el que se elija por número.
// Delega la decodificación en mpg123 (prioridad más baja) y el programa queda bloqueado leyendo stdin: 0 % CPU.
#include <algorithm>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

namespace fs = std::filesystem;

static pid_t reproduciendo = 0;

static void detener() {
    if (reproduciendo > 0) {
        kill(reproduciendo, SIGTERM);
        waitpid(reproduciendo, nullptr, 0);
        reproduciendo = 0;
    }
}

static void reproducir(const fs::path& cancion) {
    detener();
    reproduciendo = fork();
    if (reproduciendo == 0) {
        setpriority(PRIO_PROCESS, 0, 19);
        execlp("mpg123", "mpg123", "-q", cancion.c_str(), nullptr);
        _exit(127);
    }
}

int main(int argc, char** argv) {
    fs::path carpeta = argc > 1 ? fs::path(argv[1])
                                : fs::read_symlink("/proc/self/exe").parent_path() / "canciones";
    std::vector<fs::path> canciones;
    for (const auto& e : fs::directory_iterator(carpeta))
        if (e.path().extension() == ".mp3") canciones.push_back(e.path());
    std::sort(canciones.begin(), canciones.end());

    auto listar = [&] {
        std::cout << "\nCanciones:\n";
        for (size_t i = 0; i < canciones.size(); ++i)
            std::cout << "  " << i + 1 << ") " << canciones[i].stem().string() << "\n";
        std::cout << "Número = reproducir | s = detener | l = lista | q = salir\n";
    };
    listar();

    std::string linea;
    while (std::cout << "> " << std::flush, std::getline(std::cin, linea)) {
        if (reproduciendo > 0 && waitpid(reproduciendo, nullptr, WNOHANG) > 0) reproduciendo = 0;
        if (linea == "q") break;
        if (linea == "s") detener();
        else if (linea == "l") listar();
        else try {
            size_t n = std::stoul(linea);
            if (n >= 1 && n <= canciones.size()) reproducir(canciones[n - 1]);
            else std::cout << "Número fuera de rango\n";
        } catch (...) { std::cout << "Opción no válida\n"; }
    }
    detener();
}

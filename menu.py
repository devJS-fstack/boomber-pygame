import pygame
import pygame_menu

import game
from enums.algorithm import Algorithm

COLOR_BACKGROUND = (153, 153, 255)
COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)
FPS = 60.0
MENU_BACKGROUND_COLOR = (102, 102, 153)
MENU_TITLE_COLOR = (51, 51, 255)
WINDOW_SCALE = 0.75

pygame.display.init()
INFO = pygame.display.Info()
TILE_SIZE = int(INFO.current_h * 0.04)
WINDOW_SIZE = (13 * TILE_SIZE, 13 * TILE_SIZE)

clock = None
player_alg = Algorithm.PLAYER
en1_alg = Algorithm.DFS
en2_alg = Algorithm.NONE
en3_alg = Algorithm.NONE
show_path = True
surface = pygame.display.set_mode(WINDOW_SIZE)


def change_path(value, c):
    global show_path
    show_path = c


def change_player(value, c):
    global player_alg
    player_alg = c


def change_enemy1(value, c):
    global en1_alg
    en1_alg = c


def change_enemy2(value, c):
    global en2_alg
    en2_alg = c


def change_enemy3(value, c):
    global en3_alg
    en3_alg = c


def run_game():
    game.game_init(surface, show_path, player_alg,
                   en1_alg, en2_alg, en3_alg, TILE_SIZE)


def main_background():
    global surface
    surface.fill(COLOR_BACKGROUND)


def menu_loop():
    pygame.init()

    pygame.display.set_caption('Bomber Game')
    clock = pygame.time.Clock()

    menu_theme = pygame_menu.Theme(
        selection_color=COLOR_WHITE,
        widget_font=pygame_menu.font.FONT_BEBAS,
        title_font_size=TILE_SIZE,
        title_font_color=COLOR_BLACK,
        title_font=pygame_menu.font.FONT_BEBAS,
        widget_font_color=COLOR_BLACK,
        widget_font_size=int(TILE_SIZE*0.7),
        background_color=MENU_BACKGROUND_COLOR,
        title_background_color=MENU_TITLE_COLOR,
    )

    play_menu = pygame_menu.Menu(
        theme=menu_theme,
        height=int(WINDOW_SIZE[1] * WINDOW_SCALE),
        width=int(WINDOW_SIZE[0] * WINDOW_SCALE),
        title='Game Menu'
    )

    play_options = pygame_menu.Menu(
        theme=menu_theme,
        height=int(WINDOW_SIZE[1] * WINDOW_SCALE),
        width=int(WINDOW_SIZE[0] * WINDOW_SCALE),
        title='Options'
    )
    characters = [("Player", Algorithm.PLAYER), ("DFS", Algorithm.DFS),
                  ("DIJKSTRA", Algorithm.DIJKSTRA), ("None", Algorithm.NONE), ("A Star", Algorithm.A_STAR)]
    play_options.add.selector(
        "Character 1", characters, onchange=change_player)
    play_options.add.selector(
        "Character 2", characters, onchange=change_enemy1, default=1)
    play_options.add.selector(
        "Character 3", characters, onchange=change_enemy2, default=3)
    play_options.add.selector(
        "Character 4", characters, onchange=change_enemy3, default=3)
    play_options.add.selector(
        "Show path", [("Yes", True), ("No", False)], onchange=change_path)

    play_options.add.button('Back', pygame_menu.events.BACK)
    play_menu.add.button('Start', run_game)

    play_menu.add.button('Options', play_options)
    play_menu.add.button('Main menu', pygame_menu.events.BACK)

    about_menu_theme = pygame_menu.themes.Theme(
        selection_color=COLOR_WHITE,
        widget_font=pygame_menu.font.FONT_BEBAS,
        title_font_size=TILE_SIZE,
        title_font_color=COLOR_BLACK,
        title_font=pygame_menu.font.FONT_BEBAS,
        widget_font_color=COLOR_BLACK,
        widget_font_size=int(TILE_SIZE*0.5),
        background_color=MENU_BACKGROUND_COLOR,
        title_background_color=MENU_TITLE_COLOR
    )

    tutorial_menu = pygame_menu.Menu(
        theme=about_menu_theme,
        height=int(WINDOW_SIZE[1] * WINDOW_SCALE),
        width=int(WINDOW_SIZE[0] * WINDOW_SCALE),
        overflow=False,
        title='Tutorial'
    )
    tutorial_menu.add.label("Player controls: ")
    tutorial_menu.add.label("Movement: WSAD")
    tutorial_menu.add.label("Plant bomb: Space")
    tutorial_menu.add.vertical_margin(25)
    tutorial_menu.add.button('Main menu', pygame_menu.events.BACK)

    main_menu = pygame_menu.Menu(
        theme=menu_theme,
        height=int(WINDOW_SIZE[1] * WINDOW_SCALE),
        width=int(WINDOW_SIZE[0] * WINDOW_SCALE),
        onclose=pygame_menu.events.EXIT,
        title='Main menu'
    )

    main_menu.add.button('Play', play_menu)
    main_menu.add.button('Tutorial', tutorial_menu)
    main_menu.add.button('Quit', pygame_menu.events.EXIT)

    running = True
    while running:

        clock.tick(FPS)

        main_background()

        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                running = False

        if main_menu.is_enabled():
            main_menu.mainloop(surface, main_background)

        pygame.display.flip()

    exit()


if __name__ == "__main__":
    menu_loop()

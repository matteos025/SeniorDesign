import pygame
from abc import ABC
import time
from PythonSims.Vehicle import Car
from PythonSims.CarController import KeyBoardController
import numpy as np

class Vehicle_Visual(ABC):
    def __init__(self, vehicle, screen):
        pygame.sprite.Sprite.__init__(self)
        self.screen = screen
        self.vehicle = vehicle
        self.width = 50
        self.length = 100
        self.originalImage =pygame.image.load(
            "red_car.png").convert_alpha()
        self.originalImage = pygame.transform.scale(
            self.originalImage, (self.length, self.width))
        self.image = self.originalImage.copy()
        self.rect = self.image.get_rect()
        self.rect.center = (0, 0)



    def update(self, dt):
        self.vehicle.propogate(dt)

    def plot(self):
        oldCenter = self.rect.center
        car_img = self.originalImage.copy()
        self.image = pygame.transform.rotate(
            car_img, (-self.vehicle.psi * 360 / (2 * np.pi)))
        self.rect = self.image.get_rect()
        self.rect.center = oldCenter
        w, h = self.image.get_size()
        self.screen.blit(
            self.image, (self.vehicle.pos[0] - w / 2, self.vehicle.pos[1] - h / 2))


if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode(size = (1400, 600))
    background = pygame.Surface(screen.get_size())
    background.fill((0, 0, 0))
    dt = 0.1

    # loop til done
    done = False

    clock = pygame.time.Clock()
    controller = KeyBoardController()
    car = Car(controller, None, None)
    visualizer = Vehicle_Visual(car, screen)

    while not done:
        keys = pygame.key.get_pressed()  # checking pressed keys
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    controller.cur_action = 'forward'
                if event.key == pygame.K_DOWN:
                    controller.cur_action = 'backward'
                if keys[pygame.K_LEFT]:
                    controller.cur_action = 'left'
                if keys[pygame.K_RIGHT]:
                    controller.cur_action = 'right'
                if keys[pygame.K_e]:
                    print("Detected e")
                    done = True

        car.input_control()
        car.propogate(dt)
        screen.fill((0,0,0))
        visualizer.plot()

        # --- Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

        # --- Limit to 60 frames per second
        rate = 10
        clock.tick(rate)

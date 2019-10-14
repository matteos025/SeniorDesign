import pygame
from abc import ABC
import time
from Vehicle import Car
from CarController import KeyBoardController
import numpy as np


class Vehicle_Visual(ABC):
    def __init__(self, vehicle, screen):
        pygame.sprite.Sprite.__init__(self)
        self.screen = screen
        self.vehicle = vehicle
        self.width = 50
        self.length = 100
        self.originalImage = pygame.image.load(
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
            self.image,
            (self.vehicle.pos[0] - w / 2,
             self.vehicle.pos[1] - h / 2))


class CarCircleVisualizer(object):
    def __init__(self, num_cars, screen):
        pygame.sprite.Sprite.__init__(self)
        initial_vel = np.array((200., 0))
        controller = KeyBoardController()
        self.center_of_circle = np.array(
            (screen.get_width() / 2, screen.get_height() / 2))
        self.radius = 200

        self.cars = [
            Car(
                controller,
                None,
                None,
                initial_position=self.center_of_circle +
                self.radius *
                np.array(
                    [
                        np.cos(
                            2 *
                            np.pi *
                            i /
                            num_cars),
                        np.sin(
                            2 *
                            np.pi *
                            i /
                            num_cars)]),
                initial_heading=2 *
                np.pi *
                i /
                (num_cars) +
                np.pi /
                2,
                initial_velocity=initial_vel) for i in range(num_cars)]
        for car in self.cars:
            car.steering_angle = 0.01
        self.cars_viz = [
            Vehicle_Visual(
                self.cars[i],
                screen) for i in range(num_cars)]

    def update(self, dt):
        for car in self.cars_viz:
            car.update(dt)

    def plot(self):
        for car in self.cars_viz:
            car.plot()
        pygame.draw.circle(
            screen, (0, 255, 0), (int(
                self.center_of_circle[0]), int(
                self.center_of_circle[1])), self.radius, 2)


if __name__ == '__main__':
    pygame.init()
    screen_width, screen_height = 1400, 900
    screen = pygame.display.set_mode(size=(screen_width, screen_height))
    background = pygame.Surface(screen.get_size())
    background.fill((0, 0, 0))
    dt = 0.1
    circle = CarCircleVisualizer(6, screen)

    # loop til done
    done = False

    clock = pygame.time.Clock()

    while not done:
        screen.fill((255, 255, 255))
        circle.update(dt)
        circle.plot()

        # --- Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

        # --- Limit to 60 frames per second
        rate = 10
        clock.tick(rate)


if __name__ == '__main__2':
    pygame.init()
    screen_width, screen_height = 1400, 900
    screen = pygame.display.set_mode(size=(screen_width, screen_height))
    background = pygame.Surface(screen.get_size())
    background.fill((0, 0, 0))
    dt = 0.1

    # loop til done
    done = False

    clock = pygame.time.Clock()
    controller = KeyBoardController()
    center = np.array((screen_width / 2., screen_height / 2))
    pm = 300
    vel = 200
    steering_angle = 0.007
    car1 = Car(controller, None, None, initial_velocity=(vel, 0),
               initial_position=center + np.array([-pm, 0.]))
    car2 = Car(controller, None, None, initial_velocity=(
        vel, 0), initial_position=center + np.array([pm, 0.]))
    car3 = Car(controller, None, None, initial_velocity=(vel, 0),
               initial_position=center + np.array([0., -pm]))
    car4 = Car(controller, None, None, initial_velocity=(
        vel, 0), initial_position=center + np.array([0., pm]))
    car1.steering_angle = steering_angle
    car2.steering_angle = steering_angle
    car3.steering_angle = steering_angle
    car4.steering_angle = steering_angle
    car1.psi = -np.pi / 2
    car2.psi = np.pi / 2
    car3.psi = 0
    car4.psi = np.pi

    visualizer1 = Vehicle_Visual(car1, screen)
    visualizer2 = Vehicle_Visual(car2, screen)
    visualizer3 = Vehicle_Visual(car3, screen)
    visualizer4 = Vehicle_Visual(car4, screen)

    while not done:
        keys = pygame.key.get_pressed()  # checking pressed keys
        for event in pygame.event.get():
            print(event)
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

        screen.fill((255, 255, 255))

        car1.input_control()
        car1.propogate(dt)
        visualizer1.plot()

        car2.input_control()
        car2.propogate(dt)
        visualizer2.plot()

        car3.input_control()
        car3.propogate(dt)
        visualizer3.plot()

        car4.input_control()
        car4.propogate(dt)
        visualizer4.plot()

        # --- Go ahead and update the screen with what we've drawn.
        pygame.display.flip()

        # --- Limit to 60 frames per second
        rate = 10
        clock.tick(rate)

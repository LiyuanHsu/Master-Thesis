import pygame
file = 'tos_scanner.wav'
pygame.init()
pygame.mixer.init()
pygame.mixer.music.load(file)
pygame.mixer.music.play()

# pygame.event.wait()
#
while pygame.mixer.music.get_busy():
    pygame.time.Clock().tick(10)

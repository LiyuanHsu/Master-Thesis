#!/usr/bin/env python
import pygame
import re
import os

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty


class ThemeMusic(object):
    """
    Play fun theme music depending on what actions are dispatched!
    """

    def __init__(self):
        # Datastructures
        self.activity_music = None  # Mapping from string name to list of music objects
        self.activity_music_index = 0

        self.event_music = None
        self.event_music_index = 0

        self.current_music = None
        self.current_activity_id = None
        self.stopped = False
        # self.playlist = playlist
        # self.music_directory = music_directory
        # pygame.
        # Initialize pygame
        pygame.mixer.init()

        # Load music
        self._load_music()

        # ROS subscribers
        rospy.Subscriber('sound_effects',
                         String, self.play_sound_event)
        # rospy.Subscriber('/enterprise/pike/activity_start',
        #                  ActivityStart, self._activity_start_callback)
        # rospy.Subscriber('/enterprise/pike/activity_end',
        #                  ActivityEnd, self._activity_end_callback)
        # rospy.Subscriber(rospy.get_param(
        #     '~events_topic', '/theme_music_events'), String, self._event_callback)
        # TODO: services for pausing / unpausing

    def _activity_start_callback(self, msg):
        """
        Called whenever a new activty starts. Starts playing any appropriate theme music!
        """
        # Play a new song!
        # Only play if we're not already playing something (that would sound
        # really bad...)
        if self.current_music != None:
            return
        # Don't play if we're stopped
        if self.stopped:
            return
        # Select which song to play!
        self.current_music = self.choose_song_based_on_activity(msg)
        if self.current_music:
            # Begin playing it
            rospy.loginfo('Playing music for {}'.format(msg.activity))
            self.current_activity_id = msg.id
            self.current_music.play()

    def _activity_end_callback(self, msg):
        """
        Called whenever an activity ends. Stops playing the music.
        """
        # Stop the currently-playing music
        if self.current_music != None:
            if self.current_activity_id == msg.id:
                self.current_music.fadeout(2000)  # Nice 2 second fadeout
                self.current_music = None

    def play_sound_event(self, msg):
        print(msg.data)
        if msg.data in self.event_music:
            print('music found')
            sound = self.event_music[msg.data]
            self.current_music = sound
            self.current_music.play()

    def _event_callback(self, msg):
        """
        Called whenever an event happens! Doesn't interfere with other music,
        but plays over it.
        """
        # Don't play if we're stopped
        if self.stopped:
            return
        music = self.choose_song_based_on_event(msg)
        if music:
            rospy.loginfo('Playing music for event {}'.format(msg.data))
            music.play()

    def _load_music(self):
        # Load all musics
        self.activity_music = {}
        self.event_music = {}
        self.event_music['scanning'] = pygame.mixer.Sound('tos_scanner.wav')
        self.event_music['geofence'] = pygame.mixer.Sound(
            'console_explosion.wav')
        self.event_music['landing_beep'] = pygame.mixer.Sound(
            'computerbeep.wav')

        # if 'activity_names' in self.playlist:
        #     activities = self.playlist['activity_names']
        #     for activity_name in activities:
        #         self.activity_music[activity_name] = [pygame.mixer.Sound(os.path.join(self.music_directory, filename)) for filename in activities[activity_name]]
        # if 'activities_exact' in self.playlist:
        #     activities = self.playlist['activities_exact']
        #     for activity in activities:
        #         self.activity_music[activity] = pygame.mixer.Sound(os.path.join(self.music_directory, activities[activity]))
        # if 'events' in self.playlist:
        #     events = self.playlist['events']
        #     for event in events:
        # self.event_music[event] =
        # pygame.mixer.Sound(os.path.join(self.music_directory, events[event]))

    def choose_song_based_on_activity(self, msg):
        """
        Selects a song (a music object) based on the activity message.
        """
        tokens = re.split('[\s]+', msg.activity.lower().strip(' ()'))
        activity_name = tokens[0]
        if activity_name in self.activity_music:
            music_choices = self.activity_music[activity_name]
            if len(music_choices) > 0:
                music = music_choices[self.activity_music_index %
                                      len(music_choices)]
                self.activity_music_index += 1
                return music

    def choose_song_based_on_event(self, msg):
        """
        Selects a song (a music object) based on the activity message.
        """
        event = msg.data
        if event in self.event_music:
            return self.event_music[event]

    def _pause_callback(self, msg):
        """
        Stop any current music, and don't play more again until we unpause.
        """
        self.stopped = True
        if self.current_music:
            self.current_music.fadeout(2000)

    def _unpause_callback(self, msg):
        """
        Allow further music to be played.
        """
        self.stopped = False

    def go(self):
        rospy.loginfo("Theme music is ready!")
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('theme_music_player')
    # playlist=rospy.get_param('~playlist', {})
    # music_directory=rospy.get_param('~music_directory', '')
    # tm=ThemeMusic(playlist=playlist, music_directory=music_directory)
    tm = ThemeMusic()
    tm.go()

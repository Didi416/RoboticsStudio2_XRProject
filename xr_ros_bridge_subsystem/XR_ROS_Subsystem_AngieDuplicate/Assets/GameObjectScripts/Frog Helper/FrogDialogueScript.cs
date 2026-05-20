using UnityEngine;
using System.Collections.Generic;

[System.Serializable]
public class DialogueLine
{
    [TextArea(2, 4)]
    public string text;
    public AudioClip audioClip; // optional pre-recorded override
}

[System.Serializable]
public class DialogueSequence
{
    public string sequenceName;
    public List<DialogueLine> lines = new List<DialogueLine>();
}

[CreateAssetMenu(fileName = "FrogDialogue", menuName = "Frog Guide/Dialogue Data")]
public class FrogDialogue : ScriptableObject
{
    [Header("Intro Sequence")]
    public List<DialogueLine> introLines = new List<DialogueLine>()
    {
        new DialogueLine { text = "Ribbit! Oh thank goodness you are here! My name is Lily, and my frog friends are in trouble!" },
        new DialogueLine { text = "An evil force has trapped them behind puzzles that only a clever human and a robot arm can solve!" },
        new DialogueLine { text = "You will need to explore the puzzle rooms around you and find clues to help free them." },
        new DialogueLine { text = "Use the robot arm in the main room to interact with the puzzles. Point your RIGHT controller at the panel and pull the trigger to open the robot GUI." },
        new DialogueLine { text = "Good luck! Press the help button on your controller any time you need me. Now go save my friends! Ribbit!" }
    };

    [Header("Main Room Hints")]
    public List<DialogueLine> mainRoomLines = new List<DialogueLine>()
    {
        new DialogueLine { text = "Use your RIGHT controller and point it at the panel on the robot. Pull the trigger to open the control interface!" },
        new DialogueLine { text = "The GUI panel shows buttons for each puzzle. Press the puzzle number to move the robot arm to that puzzle!" },
        new DialogueLine { text = "Reset Position brings the robot back to its starting spot. Reset Puzzle starts that puzzle over from scratch." }
    };

    [Header("Puzzle Room 1 - Button Matrix Hints")]
    public List<DialogueLine> puzzle1Hints = new List<DialogueLine>()
    {
        new DialogueLine { text = "Hm... I wonder what that sign on the right says... something about following the cycle of life?" },
        new DialogueLine { text = "My froggy friends jump in a certain order... maybe the numbers on the signs around the room hold a clue?" },
        new DialogueLine { text = "Listen carefully... each frog croaks a different number of times. Could those numbers be a code?" }
    };

    [Header("Puzzle Room 2 - Slider Hints")]
    public List<DialogueLine> puzzle2Hints = new List<DialogueLine>()
    {
        new DialogueLine { text = "My friends love jumping on lily pads! Can you follow their favourite path? Maybe try touching one?" },
        new DialogueLine { text = "The lily pads light up in a special order... watch carefully and remember the sequence!" },
        new DialogueLine { text = "The robot arm needs to follow the same path the lily pads showed you. Start from the top left corner!" }
    };

    [Header("Puzzle Room 3 - Egg Sorting Hints")]
    public List<DialogueLine> puzzle3Hints = new List<DialogueLine>()
    {
        new DialogueLine { text = "Oh! Those are my friend's eggs! Can you pick them up? Try pointing at one and pulling the trigger!" },
        new DialogueLine { text = "Each egg belongs on a special stand... look carefully at the symbols. Water goes with water, fire with fire!" },
        new DialogueLine { text = "The stands have been moved around to trick you! Look around the room carefully for where they are hiding!" }
    };
}

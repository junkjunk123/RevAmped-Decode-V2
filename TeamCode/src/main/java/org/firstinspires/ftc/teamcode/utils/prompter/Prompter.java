package org.firstinspires.ftc.teamcode.utils.prompter;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.GamepadEx;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

//Modified From the Marrow Library
public class Prompter {
    private final OpMode opmode;
    private final GamepadEx gamepadInput;

    private final List<KeyPromptPair<?>> prompts = new ArrayList<>();
    private final Map<String, Object> results = new HashMap<>();

    private Runnable completeFunc;

    private int currentIndex = 0;
    private boolean isCompleted = false;

    private String finalMessage;
    private Supplier<String> finalMessageSupplier;

    public Prompter(OpMode opMode, GamepadEx gamepad) {
        this.opmode = opMode;
        this.gamepadInput = gamepad;
    }

    /**
     * Add a prompt to the queue.
     */
    public <T> Prompter prompt(String key, Prompt<T> prompt) {
        prompts.add(new KeyPromptPair<>(key, () -> prompt));
        return this; // For method chaining
    }

    /**
     * Add a prompt to the queue.
     */
    public <T> Prompter prompt(String key, Supplier<Prompt<T>> promptSupplier) {
        prompts.add(new KeyPromptPair<>(key, promptSupplier));
        return this; // For method chaining
    }


    /**
     * Gets the chosen value of a prompt from its key.
     *
     * @param key The prompt's key
     * @return The value of the prompt result
     */
    @SuppressWarnings("unchecked")
    public <T> T get(String key) {
        return (T) results.get(key);
    }

    /**
     * Gets the chosen value of a prompt from its key.
     *
     * @param key The prompt's key
     * @param defaultValue A default value if the value doesn't exist
     * @return The value of the prompt result
     */
    @SuppressWarnings("unchecked")
    public <T> T getOrDefault(String key, T defaultValue) {
        return (T) results.getOrDefault(key, defaultValue);
    }

    /**
     * Runs all queued prompts until all prompts are complete.
     * This should be called in a loop.
     */
    public void run() {
        if (isCompleted && finalMessage != null) {
            opmode.telemetry.addLine(finalMessage);
            opmode.telemetry.update();
            return;
        }

        boolean finished = processPrompts();
        opmode.telemetry.update();

        if (finished) {
            isCompleted = true;
            if (completeFunc != null) completeFunc.run();

            if (finalMessageSupplier != null)
                finalMessage = finalMessageSupplier.get();
        }
    }

    /**
     * Sets a function to run once all prompts are complete.
     * @param func The function to run
     */
    public Prompter onComplete(Runnable func) {
        completeFunc = func;
        return this; // For method chaining
    }

    /**
     * Handles the prompts and inputs. Should be called in a loop.
     * @return True if there are no more prompts to process, false otherwise
     */
    private boolean processPrompts() {
        gamepadInput.update();
        // Handle back navigation
        if (gamepadInput.b.isRisingEdge() && currentIndex > 0) {
            navigateBack();
        }

        // No prompts left
        if (currentIndex >= prompts.size()) {
            return true;
        }

        KeyPromptPair<?> current = prompts.get(currentIndex);
        Prompt<?> prompt = current.getPrompt();

        // Skip if prompt is null
        if (prompt == null) {
            currentIndex++;
            return false;
        }

        Object result = prompt.process();
        if (result != null) {
            results.put(current.getKey(), result);
            currentIndex++;
        }

        return false;
    }

    private void navigateBack() {
        do {
            prompts.get(currentIndex).reset(); // Reset prompt so it will get a fresh prompt every time
            currentIndex--;
        } while (prompts.get(currentIndex).getPrompt() == null && currentIndex > 0); // Skip all null prompts
    }

    private class KeyPromptPair<T> {
        private final String key;
        private final Supplier<Prompt<T>> promptSupplier;

        private Prompt<T> prompt;

        public KeyPromptPair(String key, Supplier<Prompt<T>> promptSupplier) {
            this.key = key;
            this.prompt = null;
            this.promptSupplier = promptSupplier;
        }

        public String getKey() {
            return key;
        }

        public Prompt<T> getPrompt() {
            if (prompt != null) return prompt;
            if (promptSupplier == null) return null;

            // Save the created prompt so it doesn't reset every loop
            prompt = promptSupplier.get();

            if (prompt != null) {
                prompt.configure(gamepadInput, opmode.telemetry);
            }

            return prompt;
        }

        public void reset() {
            // Only clear if it's a supplier-based prompt
            if (promptSupplier != null) {
                prompt = null;
            }
        }
    }

    public Prompter thenDisplay(String line) {
        this.finalMessage = line;
        return this;
    }

    public Prompter thenDisplay(Supplier<String> finalMessageSupplier) {
        this.finalMessageSupplier = finalMessageSupplier;
        return this;
    }

    public boolean isCompleted() {
        return isCompleted;
    }
}
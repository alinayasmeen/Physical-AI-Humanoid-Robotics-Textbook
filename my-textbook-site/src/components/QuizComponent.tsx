import React, { useState } from 'react';
import styles from './QuizComponent.module.css';

interface Question {
  question: string;
  options: string[];
  correctIndex: number;
  explanation: string;
}

interface QuizComponentProps {
  questions: Question[];
}

const QuizComponent: React.FC<QuizComponentProps> = ({ questions }) => {
  const [userAnswers, setUserAnswers] = useState<Array<number | null>>(
    Array(questions.length).fill(null)
  );
  const [submitted, setSubmitted] = useState(false);

  const handleAnswerSelect = (questionIndex: number, optionIndex: number) => {
    if (submitted) return; // Don't allow changes after submission

    const newAnswers = [...userAnswers];
    newAnswers[questionIndex] = optionIndex;
    setUserAnswers(newAnswers);
  };

  const handleSubmit = () => {
    setSubmitted(true);
  };

  const handleReset = () => {
    setUserAnswers(Array(questions.length).fill(null));
    setSubmitted(false);
  };

  const getOptionClass = (questionIndex: number, optionIndex: number) => {
    const userAnswer = userAnswers[questionIndex];
    const isCorrect = optionIndex === questions[questionIndex].correctIndex;
    const isSelected = userAnswer === optionIndex;

    if (submitted) {
      if (isCorrect) {
        return styles.correctOption;
      } else if (isSelected && !isCorrect) {
        return styles.incorrectOption;
      }
    }

    return isSelected ? styles.selectedOption : '';
  };

  return (
    <div className={styles.quizContainer}>
      <h3>Knowledge Check</h3>
      {questions.map((question, qIndex) => (
        <div key={qIndex} className={styles.questionContainer}>
          <p className={styles.questionText}>{qIndex + 1}. {question.question}</p>
          <div className={styles.optionsContainer}>
            {question.options.map((option, oIndex) => (
              <div
                key={oIndex}
                className={`${styles.option} ${getOptionClass(qIndex, oIndex)}`}
                onClick={() => handleAnswerSelect(qIndex, oIndex)}
              >
                <span className={styles.optionLetter}>
                  {String.fromCharCode(65 + oIndex)}.
                </span>{' '}
                <span className={styles.optionText}>{option}</span>
              </div>
            ))}
          </div>
          {submitted && userAnswers[qIndex] !== null && (
            <div className={styles.feedback}>
              <div className={styles.explanation}>
                <strong>Explanation:</strong> {question.explanation}
              </div>
              <div className={styles.answerResult}>
                {userAnswers[qIndex] === question.correctIndex ? (
                  <span className={styles.correct}>✓ Correct!</span>
                ) : (
                  <span className={styles.incorrect}>
                    ✗ Incorrect. The correct answer is: {String.fromCharCode(65 + question.correctIndex)}.
                  </span>
                )}
              </div>
            </div>
          )}
        </div>
      ))}
      <div className={styles.buttonContainer}>
        {!submitted ? (
          <button className={styles.submitButton} onClick={handleSubmit}>
            Submit Answers
          </button>
        ) : (
          <button className={styles.resetButton} onClick={handleReset}>
            Reset Quiz
          </button>
        )}
      </div>
    </div>
  );
};

export default QuizComponent;
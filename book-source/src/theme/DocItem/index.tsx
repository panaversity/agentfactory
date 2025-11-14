import React, { useState, useEffect } from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import QuizModal from '@site/src/components/QuizModal';
import allQuizData from '@site/src/merged-quiz-data.json';
import '@site/src/css/quiz-modal.css';
import LessonTabs from '@site/src/components/LessonTabs/LessonTabs';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  const [isQuizOpen, setIsQuizOpen] = useState(false);
  const [chapterId, setChapterId] = useState<number | null>(null);

  const docTitle = props.content?.metadata?.title;

  useEffect(() => {
    // Find the chapter in the quiz data by matching the document title
    const matchedChapter = docTitle
      ? allQuizData.quiz_data.chapters.find(
          (chapter) => chapter.chapter_title === docTitle
        )
      : undefined;

    if (matchedChapter) {
      setChapterId(matchedChapter.chapter_id);
    } else {
      setChapterId(null);
    }
  }, [docTitle]); // Re-run this effect when the document title changes

  // Find the specific quiz for the current chapter
  const chapterQuiz = chapterId
    ? allQuizData.quiz_data.chapters.find(
        (chapter) => chapter.chapter_id === chapterId
      )
    : undefined;

  // Determine if a valid, non-skipped quiz with questions exists
  const hasQuiz = chapterQuiz && !chapterQuiz.skipped && (chapterQuiz.questions?.length ?? 0) > 0;

  const handleOpenQuiz = () => {
    if (hasQuiz) {
      setIsQuizOpen(true);
    } else {
      alert('A quiz for this chapter is not available yet.');
    }
  };

  const handleCloseQuiz = () => {
    setIsQuizOpen(false);
  };

  return (
    <>
      <LessonTabs content={props.content} onOpenQuiz={handleOpenQuiz} />
      <DocItem {...props} />

      {isQuizOpen && chapterId && (
        <QuizModal
          chapterId={chapterId}
          onClose={handleCloseQuiz}
          quizData={allQuizData.quiz_data.chapters}
        />
      )}
    </>
  );
}

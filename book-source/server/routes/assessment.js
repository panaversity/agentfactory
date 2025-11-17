const express = require('express');
const { generateAssessment } = require('../services/assessmentService');

const router = express.Router();

router.post('/generate', async (req, res) => {
  try {
    const {
      questionCount = 5,
      difficulty = 'medium',
      topic = 'AI Native Software Development',
      examType = 'General Assessment',
    } = req.body || {};

    const parsedCount = Number(questionCount);
    if (Number.isNaN(parsedCount) || parsedCount <= 0) {
      return res.status(400).json({ error: 'questionCount must be a positive number' });
    }

    const payload = await generateAssessment({
      questionCount: parsedCount,
      difficulty: String(difficulty).toLowerCase(),
      topic,
      examType,
    });

    res.json(payload);
  } catch (error) {
    console.error('Assessment generation failed:', error);
    res.status(500).json({
      error: 'Failed to generate assessment',
      message: error.message,
    });
  }
});

module.exports = { assessmentRouter: router };



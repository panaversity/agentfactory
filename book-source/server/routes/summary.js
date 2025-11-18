const express = require('express');
const { generateSummary, getSummary } = require('../services/summaryService');

const router = express.Router();

// Generate summary for a specific page
router.post('/generate', async (req, res) => {
  try {
    console.log('\n📨 [Backend Route] Received generate request');
    console.log('📦 [Backend Route] Request body:', req.body);

    const { pagePath, pageTitle, size = 'medium' } = req.body;

    console.log('🔍 [Backend Route] Extracted values:', {
      pagePath,
      pageTitle,
      size,
    });

    if (!pagePath || typeof pagePath !== 'string') {
      return res.status(400).json({
        error: 'pagePath is required and must be a string'
      });
    }

    if (!pageTitle || typeof pageTitle !== 'string') {
      return res.status(400).json({
        error: 'pageTitle is required and must be a string'
      });
    }

    // Validate size parameter
    const validSizes = ['short', 'medium', 'long'];
    if (size && !validSizes.includes(size)) {
      return res.status(400).json({
        error: 'size must be one of: short, medium, long'
      });
    }

    console.log('🚀 [Backend Route] Calling generateSummary with size:', size);

    const summary = await generateSummary(pagePath, pageTitle, size);

    console.log('✅ [Backend Route] Summary generated successfully');
    console.log('📊 [Backend Route] Summary length:', summary.length);

    res.json({
      success: true,
      summary,
      pagePath,
      size
    });
  } catch (error) {
    console.error('Error generating summary:', error);
    res.status(500).json({
      error: 'Failed to generate summary',
      message: error.message
    });
  }
});

// Check if summary exists for a page
router.get('/check', async (req, res) => {
  try {
    const { pagePath, size = 'medium' } = req.query;

    if (!pagePath || typeof pagePath !== 'string') {
      return res.status(400).json({
        error: 'pagePath is required and must be a string'
      });
    }

    // Validate size parameter
    const validSizes = ['short', 'medium', 'long'];
    if (size && !validSizes.includes(size)) {
      return res.status(400).json({
        error: 'size must be one of: short, medium, long'
      });
    }

    const summary = await getSummary(pagePath, size);

    res.json({
      exists: !!summary,
      summary: summary || null,
      pagePath,
      size
    });
  } catch (error) {
    console.error('Error checking summary:', error);
    res.status(500).json({
      error: 'Failed to check summary',
      message: error.message
    });
  }
});

module.exports = { summaryRouter: router };

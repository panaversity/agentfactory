// Simple script to update admin role using fetch to the Neon SQL API
const { neon } = require('@neondatabase/serverless');
require('dotenv').config({ path: '.env.local' });

const sql = neon(process.env.DATABASE_URL);

async function setAdminRole() {
  try {
    const result = await sql`
      UPDATE "user"
      SET role = 'admin'
      WHERE email = 'admin@robolearn.io'
      RETURNING id, email, role
    `;
    console.log('✅ Admin role set successfully!');
    console.log('Result:', result);
  } catch (error) {
    console.error('Error:', error);
  }
}

setAdminRole();

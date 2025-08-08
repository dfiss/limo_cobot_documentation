import React from 'react';
import {translate} from '@docusaurus/Translate';

export default function LastUpdated({
  lastUpdatedAt,
  lastUpdatedBy,
}) {
  // Always show our custom date and author
  const customDate = 'August 8, 2025';
  const customAuthor = 'KRISH RAY';
  
  return (
    <span className="theme-last-updated">
      <b>
        {translate({
          id: 'theme.lastUpdated.atDate',
          message: ' on {date}',
          description: 'The words used to describe on which date a page has been last updated',
        }).replace('{date}', customDate)}
      </b>
      {customAuthor && (
        <>
          {' '}
          <b>
            {translate({
              id: 'theme.lastUpdated.byUser',
              message: 'by {user}',
              description: 'The words used to describe by who the page has been last updated',
            }).replace('{user}', customAuthor)}
          </b>
        </>
      )}
      {process.env.NODE_ENV === 'development' && (
        <div>
          <small style={{color: '#888'}}>
            (Simulated during dev for better perf)
          </small>
        </div>
      )}
    </span>
  );
}
